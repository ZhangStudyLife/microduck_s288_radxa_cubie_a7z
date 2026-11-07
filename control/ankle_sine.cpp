// 外部控制程序示例：50Hz发送目标，right_knee做0.5Hz、±30度的正弦往复。
// 仅通过robotd的Unix Socket发命令，不打开串口，也不启动robotctl子进程。
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

volatile std::sig_atomic_t running = 1;
void interrupt(int) { running = 0; }

// robotd每个连接处理一条命令，因此每次更新重新连接本机IPC。
bool request(const std::string& command) {
    int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (fd < 0) { perror("创建IPC连接失败"); return false; }
    timeval timeout{1, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::snprintf(address.sun_path, sizeof(address.sun_path),
                  "/tmp/microduck-%u/robot.sock", unsigned(getuid()));
    if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0
        || send(fd, command.data(), command.size(), MSG_NOSIGNAL) != ssize_t(command.size())) {
        perror("发送IPC命令失败"); close(fd); return false;
    }
    char reply[1024];
    ssize_t n = recv(fd, reply, sizeof(reply), 0);
    close(fd);
    if (n <= 0) { std::fprintf(stderr, "未收到robotd响应：%s\n", command.c_str()); return false; }
    std::string result(reply, size_t(n));
    if (result.rfind("OK", 0) != 0) {
        // IGNORED也结束本次测试，避免目标未被接受却误认为正在正常往复。
        std::fprintf(stderr, "%s -> %s\n", command.c_str(), result.c_str()); return false;
    }
    return true;
}

int main() {
    using Clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    if (!request("joint arm right_knee")) {
        request("joint stop right_knee"); return 1;
    }
    puts("right_knee已使能：50Hz发送，目标=30*sin(2*pi*0.5*t)度；Ctrl+C停止。");
    fflush(stdout);
    const auto start = Clock::now();
    auto next = start;
    unsigned sent = 0, late = 0;
    bool ok = true;
    while (running) {
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        // 0.5Hz是往复频率：2秒一周期；50Hz是目标更新频率：20ms一条。
        // 这是绝对机械角，中心为标定零点，不是相对启动位置偏移±30度。
        double angle = 30.0 * std::sin(2.0 * 3.141592653589793 * 0.5 * t);
        if (!request("joint move right_knee " + std::to_string(angle))) { ok = false; break; }
        ++sent;
        next += 20ms;
        // 用绝对时间调度，避免把IPC耗时不断累加到周期；迟到时跳过，不突发补发。
        if (Clock::now() > next) {
            ++late;
            do { next += 20ms; } while (next < Clock::now());
        }
        std::this_thread::sleep_until(next);
    }
    // 退出外部程序不会自动停止robotd，必须主动发送stop；模式0仍不是撤力。
    bool stopped = request("joint stop right_knee");
    std::printf("已发送%u条目标，超期%u次；%s\n", sent, late,
                stopped ? "已请求模式0锁定" : "停止请求失败，请检查robotd");
    return ok && stopped ? 0 : 1;
}
