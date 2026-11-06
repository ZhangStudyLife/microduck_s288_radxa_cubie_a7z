#pragma once
#include <cstdint>
#include <chrono>

// 已换算为物理单位的反馈。rotor 字段均为转子端，不能当作输出轴数值。
struct S288State {
    unsigned id{}, mode{}, timeout{}, winding_temp{}, warning{};
    int case_temp{};
    double voltage{}, rotor_torque{}, rotor_speed{}, rotor_position{}, output_degrees{};
    uint32_t error{};
};

// 控制参数全部使用转子端物理单位；默认锁定并开启超时保护。
// Foc 模式下五个参数共同生效，不是五种互斥的控制模式。
struct S288Command {
    enum class Mode { Lock = 0, Foc = 1 };
    Mode mode = Mode::Lock;
    bool timeout_protection = true; // false 同时具有清除超时标志的协议含义
    double torque = 0;   // 前馈力矩 N*m，编码比例256000
    double speed = 0;    // rad/s，编码比例2.56/(2*pi)
    double position = 0; // rad，编码比例32768/(2*pi)
    double kp = 0;       // N*m/rad，编码比例1280000，要求非负
    double kd = 0;       // N*m/(rad/s)，编码比例128000000，要求非负
};

// 一条串口总线对应一个对象，多台电机共用它，顺序调用，不能并发访问。
class S288Bus {
public:
    enum class Result { Ok, Timeout, Error, Interrupted, InvalidArgument };
    S288Bus() = default;
    ~S288Bus(); // 对象离开作用域时自动关闭串口，不发送电机命令。
    S288Bus(const S288Bus&) = delete; // 禁止复制，避免两个对象关闭同一个串口。
    S288Bus& operator=(const S288Bus&) = delete;

    bool open(const char* device); // 固定设置 6 Mbps、8N1；失败可用 perror 查看原因。
    void close();
    // 向 ID 0~14 发模式0锁定命令，再等待反馈，默认收发总预算1ms（需实测确认硬件响应）。
    // 只有 Ok 时才更新 state；其余结果不得把旧 state 当作新反馈。
    // 此接口会控制电机进入锁定模式，并非无动作的只读查询。
    Result lock_and_read(unsigned id, S288State& state);
    // 校验ID、模式、有限数和编码范围；不裁剪参数，非法参数不发送。
    // 范围是协议可表示范围，不是电机或机构的安全运动范围。
    static bool valid_command(unsigned id, const S288Command& command);
    // budget 同时约束发送和接收；OS调度可能使返回晚于预算，不是硬实时保证。
    Result exchange(unsigned id, const S288Command& command, S288State& state,
                    std::chrono::microseconds budget = std::chrono::microseconds(1000));
    // 发送模式0、timeout=0的命令；会关闭超时保护并请求清除超时标志。
    // 返回Ok只代表通信成功，调用者还需检查state.timeout和state.error。
    // 随后必须显式发送timeout_protection=true的命令以恢复保护。
    Result clear_timeout(unsigned id, S288State& state);
private:
    int fd_ = -1; // -1 表示未打开；资源由本对象独占。
};
