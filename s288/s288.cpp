#include "s288.hpp"
#include <cmath>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

namespace { // 字节处理细节仅本文件可见，调用者不需要知道协议偏移。
// 显式按小端读写，避免结构体填充、位域和未对齐访问影响协议。
uint32_t get_le(const uint8_t* p, int n)
{
    uint32_t value = 0;
    for (int i = 0; i < n; ++i) value |= uint32_t(p[i]) << (8 * i);
    return value;
}
void put_le(uint8_t* p, uint32_t value, int n)
{
    for (int i = 0; i < n; ++i) p[i] = uint8_t(value >> (8 * i));
}
// 有符号字段：最高位为 1 时，减去 2^位数，恢复负数。
int64_t signed_le(const uint8_t* p, int n)
{
    uint32_t value = get_le(p, n);
    return (value & (uint32_t(1) << (8 * n - 1)))
        ? int64_t(value) - (int64_t(1) << (8 * n)) : value;
}

// 与提供的 crc32_core 相同：先按小端取 32 位字，再从最高位处理。
// bytes 必须为 4 的倍数；发送覆盖 16 字节，反馈覆盖 20 字节。
uint32_t crc32(const uint8_t* p, int bytes)
{
    uint32_t crc = 0xffffffff;
    for (int i = 0; i < bytes; i += 4) {
        uint32_t word = get_le(p + i, 4);
        for (uint32_t bit = 0x80000000; bit; bit >>= 1) {
            bool high = (crc & 0x80000000) != 0;
            crc <<= 1;
            if (high) crc ^= 0x04c11db7;
            if (word & bit) crc ^= 0x04c11db7;
        }
    }
    return crc;
}

constexpr double pi = 3.141592653589793;
// 先检查物理量范围，再转整数，避免溢出或NaN转换；截断方向与原协议一致。
bool fits(double value, double scale, double low, double high)
{
    return std::isfinite(value) && value >= low / scale && value <= high / scale;
}
void make_command(uint8_t* tx, unsigned id, const S288Command& command)
{
    memset(tx, 0, 20);
    tx[0] = 0xfe; tx[1] = 0xee;
    tx[2] = uint8_t(id | (unsigned(command.mode) << 4)
                      | (command.timeout_protection ? 0x80u : 0u));
    // 有符号整数转uint32_t后保留低位，产生小端补码字节。
    put_le(tx + 4, uint32_t(int32_t(command.torque * 256000)), 2);
    put_le(tx + 6, uint32_t(int32_t(command.speed * (2.56 / (2 * pi)))), 2);
    put_le(tx + 8, uint32_t(int32_t(command.position * (32768 / (2 * pi)))), 4);
    put_le(tx + 12, uint32_t(int32_t(command.kp * 1280000)), 2);
    put_le(tx + 14, uint32_t(int32_t(command.kd * 128000000)), 2);
    put_le(tx + 16, crc32(tx, 16), 4);
}

bool valid_reply(const uint8_t* rx, int id)
{
    // 反馈帧头不参与 CRC；线路上不存在结构体中的 NoUse 两字节。
    return rx[0] == 0xfc && rx[1] == 0xee && (rx[2] & 15) == id
        && get_le(rx + 22, 4) == crc32(rx + 2, 20);
}

using Clock = std::chrono::steady_clock;
// ppoll 支持亚毫秒等待；发送和接收共享同一个绝对期限，避免无限等待。
S288Bus::Result wait_ready(int fd, short events, Clock::time_point deadline)
{
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - Clock::now()).count();
    if (ns <= 0) return S288Bus::Result::Timeout;
    timespec timeout{time_t(ns / 1000000000), long(ns % 1000000000)};
    pollfd event{fd, events, 0};
    int n = ppoll(&event, 1, &timeout, nullptr);
    if (n < 0) return errno == EINTR ? S288Bus::Result::Interrupted : S288Bus::Result::Error;
    if (!n) return S288Bus::Result::Timeout;
    if (event.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        errno = EIO; return S288Bus::Result::Error;
    }
    return S288Bus::Result::Ok;
}
// 批量读取后在内存中找帧，不再为每一个字节执行 poll/read。
S288Bus::Result receive_reply(int fd, uint8_t* rx, int id, Clock::time_point deadline)
{
    uint8_t buffer[256];
    int used = 0;
    while (true) {
        auto result = wait_ready(fd, POLLIN, deadline);
        if (result != S288Bus::Result::Ok) return result;
        ssize_t n = read(fd, buffer + used, sizeof(buffer) - size_t(used));
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n < 0) return errno == EINTR ? S288Bus::Result::Interrupted : S288Bus::Result::Error;
        used += int(n);
        int start = 0;
        for (; start + 26 <= used; ++start) {
            if (valid_reply(buffer + start, id)) {
                memcpy(rx, buffer + start, 26);
                return S288Bus::Result::Ok;
            }
        }
        // 保留末尾不足一帧的字节，以便和下一次读取拼接。
        used -= start;
        memmove(buffer, buffer + start, size_t(used));
    }
}

} // namespace

S288Bus::~S288Bus() { close(); }
void S288Bus::close()
{
    // 保留失败操作的 errno，让调用者能打印真正的错误原因。
    int saved = errno;
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    errno = saved;
}
bool S288Bus::open(const char* device)
{
    close();
    fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) return false;
    termios2 config{};
    if (ioctl(fd_, TCGETS2, &config) == -1) {
        close(); return false;
    }
    config.c_iflag = config.c_oflag = config.c_lflag = 0; // 原始二进制传输
    config.c_cflag &= ~(CBAUD | CSIZE | PARENB | CSTOPB | CRTSCTS);
    config.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD; // 自定义波特率、8N1
    config.c_ispeed = config.c_ospeed = 6000000;
    config.c_cc[VMIN] = config.c_cc[VTIME] = 0; // 等待期限由 poll 负责
    if (ioctl(fd_, TCSETS2, &config) == -1) {
        close(); return false;
    }
    return true;
}

bool S288Bus::valid_command(unsigned id, const S288Command& command)
{
    return id <= 14
        && (command.mode == S288Command::Mode::Lock || command.mode == S288Command::Mode::Foc)
        && fits(command.torque, 256000, -32768, 32767)
        && fits(command.speed, 2.56 / (2 * pi), -32768, 32767)
        && fits(command.position, 32768 / (2 * pi), -2147483648.0, 2147483647.0)
        && fits(command.kp, 1280000, 0, 32767)
        && fits(command.kd, 128000000, 0, 32767);
}
S288Bus::Result S288Bus::lock_and_read(unsigned id, S288State& state)
{
    return exchange(id, S288Command{}, state);
}
S288Bus::Result S288Bus::clear_timeout(unsigned id, S288State& state)
{
    S288Command command;
    command.timeout_protection = false;
    return exchange(id, command, state);
}
S288Bus::Result S288Bus::exchange(unsigned id, const S288Command& command, S288State& state,
                                      std::chrono::microseconds budget)
{
    if (!valid_command(id, command)) { errno = EINVAL; return Result::InvalidArgument; }
    if (budget.count() <= 0) { errno = EINVAL; return Result::InvalidArgument; }
    auto deadline = Clock::now() + budget;
    // 丢弃已经排队的旧反馈；协议无序号，仍无法完全区分迟到的同ID回复。
    if (ioctl(fd_, TCFLSH, TCIFLUSH) < 0) return Result::Error;
    uint8_t tx[20], rx[26];
    make_command(tx, id, command);
    for (int sent = 0; sent < 20;) {
        auto ready = wait_ready(fd_, POLLOUT, deadline);
        if (ready != Result::Ok) return ready;
        ssize_t n = ::write(fd_, tx + sent, size_t(20 - sent));
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n < 0 && errno == EINTR) return Result::Interrupted;
        if (n <= 0) { if (!n) errno = EIO; return Result::Error; }
        sent += int(n);
    }
    Result result = receive_reply(fd_, rx, int(id), deadline);
    if (result != Result::Ok) return result;

    // 完整校验通过后再更新反馈，保留所有物理单位的换算集中在此。
    constexpr double pi = 3.141592653589793;
    state.id = rx[2] & 15;
    state.mode = (rx[2] >> 4) & 7;
    state.timeout = rx[2] >> 7;
    state.case_temp = int(signed_le(rx + 3, 1));
    state.winding_temp = rx[4];
    state.voltage = rx[5] * 0.5;
    state.rotor_torque = double(signed_le(rx + 6, 2)) / 256000.0;
    state.rotor_speed = double(signed_le(rx + 8, 2)) * (2 * pi / 2.56);
    state.rotor_position = double(signed_le(rx + 10, 4)) * (2 * pi / 32768);
    state.error = get_le(rx + 14, 4);
    state.output_degrees = (get_le(rx + 18, 2) & 8191) * (360.0 / 8192);
    state.warning = get_le(rx + 18, 2) >> 13;
    return Result::Ok;
}
