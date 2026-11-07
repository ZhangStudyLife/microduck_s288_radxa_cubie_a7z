#pragma once
#include "../s288/s288.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <unistd.h>

namespace joint {
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using namespace std::chrono_literals;
constexpr double ratio=288.35, rad=3.141592653589793/180.0;
inline double wrap(double x) { return x-360.0*std::floor((x+180.0)/360.0); }
struct Config {
    std::string name;
    bool calibrated=false;
    double zero=0;
    int direction=1;
    double low=-1, high=1, kp=1, kd=0.02;
};
inline bool valid(const Config& c) {
    for (double x:{c.zero,c.low,c.high,c.kp,c.kd})
        if (!std::isfinite(x)) return false;
    // 手册第8页规定转子增益编码；输出端增益除减速比平方后复用协议校验。
    // 这里检查可编码范围，不代表该增益已适合机构承重。
    S288Command cmd; cmd.kp=c.kp/(ratio*ratio);cmd.kd=c.kd/(ratio*ratio);
    return S288Bus::valid_command(0,cmd)
        && !c.name.empty() && c.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_")==std::string::npos
        && c.zero>=0 && c.zero<360 && (c.direction==1 || c.direction==-1)
        && c.low<0 && c.high>0 && c.high-c.low<180;
}
inline std::array<Config,15> defaults() {
    std::array<Config,15> cs;
    for (unsigned i=0;i<15;++i) cs[i].name=i==0?"right_ankle":"motor_"+std::to_string(i);
    return cs;
}
// 固定列，无依赖配置框架。整份配置通过校验后才替换内存，错误时禁用所有关节。
inline bool load(const std::string& path, std::array<Config,15>& cs) {
    auto candidate=defaults(); std::array<bool,15> seen{};
    std::ifstream file(path); if (!file) return false;
    std::string line;
    while (std::getline(file,line)) {
        if (line.empty() || line[0]=='#') continue;
        std::istringstream in(line); unsigned id; int calibrated; Config c; std::string extra;
        if (!(in>>id>>c.name>>calibrated>>c.zero>>c.direction>>c.low>>c.high>>c.kp>>c.kd)
            || (in>>extra) || id>=15 || seen[id] || (calibrated!=0 && calibrated!=1) || !valid(c)) return false;
        c.calibrated=calibrated==1; candidate[id]=c; seen[id]=true;
    }
    for (unsigned i=0;i<15;++i) {
        if (!seen[i]) return false;
        for (unsigned j=0;j<i;++j) if(candidate[i].name==candidate[j].name) return false;
    }
    cs=candidate; return true;
}
inline bool save(const std::string& path, const std::array<Config,15>& cs) {
    std::ostringstream out; out<<std::setprecision(17);
    out<<"# id name calibrated zero_sensor_deg direction min_deg max_deg kp kd\n";
    for (unsigned i=0;i<15;++i) {
        const auto& c=cs[i]; if (!valid(c)) return false;
        out<<i<<' '<<c.name<<' '<<c.calibrated<<' '<<c.zero<<' '<<c.direction<<' '<<c.low<<' '<<c.high
           <<' '<<c.kp<<' '<<c.kd<<'\n';
    }
    // 同目录临时文件+fsync+rename；写入失败不更改原文件和内存配置。
    std::string tmp=path+".tmp.XXXXXX"; int fd=mkstemp(tmp.data()); if(fd<0) return false;
    std::string data=out.str(); size_t done=0; bool ok=true;
    while(done<data.size()) { ssize_t n=write(fd,data.data()+done,data.size()-done); if(n<=0){ok=false;break;} done+=size_t(n); }
    if(ok && fsync(fd)<0) ok=false;
    if(close(fd)<0) ok=false;
    if(ok && rename(tmp.c_str(),path.c_str())<0) ok=false;
    if(!ok) unlink(tmp.c_str());
    return ok;
}
// 只保存使能意图、目标、换算参考和最近反馈；不存在“动作完成/故障停用”状态机。
struct State {
    S288State feedback{};
    Time good{};
    bool enabled=false, reference=false;
    double anchor_rotor=0, anchor_joint=0, target=0;
    int stage=0; // 仅标定向导使用：1零点，2远端，3近端，4回中。
    Config draft;
    double q(const Config& c) const { return c.direction*wrap(feedback.output_degrees-c.zero); }
    bool fresh(Time now) const { return good!=Time{} && now-good<=100ms; }
    void capture(const Config& c) {
        anchor_rotor=feedback.rotor_position;anchor_joint=q(c);target=anchor_joint;reference=true;
    }
    void stop() { enabled=false;reference=false;target=0;stage=0; }
    // 掉线只作废旧目标/参考，不撤销用户的使能意图；新反馈到来时保持新当前位置。
    void disconnect() { good={};reference=false;target=0; }
    void observe(const S288State& f, Time now, const Config& c) {
        const bool reset=(!feedback.timeout && f.timeout)
            || (!(feedback.error & 0x4000u) && (f.error & 0x4000u)); // 手册：bit14异常复位。
        if(reset) {reference=false;target=0;}
        feedback=f;good=now;
        if(enabled && !reference) capture(c);
    }
    std::string arm(const Config& c, Time now) {
        if(stage) return "ERR finish or cancel calibration first";
        if(enabled) return "OK already enabled; target unchanged";
        enabled=true;reference=false;
        if(fresh(now)) capture(c);
        return "OK enabled; hold current position when feedback is available";
    }
    std::string move(const Config& c, double value) {
        if(!std::isfinite(value)) return "IGNORED invalid angle; target unchanged";
        if(!c.calibrated) return "ERR calibrate this joint first";
        if(value<c.low || value>c.high) return "IGNORED angle outside limits; target unchanged";
        if(!enabled) return "ERR joint not enabled; use joint arm NAME";
        if(!reference) return "ERR waiting for feedback; no target buffered";
        // 先验证转换后的协议字段，再替换唯一目标，非法请求不破坏旧目标。
        S288Command cmd;
        cmd.position=anchor_rotor+c.direction*(value-anchor_joint)*rad*ratio;
        if(!S288Bus::valid_command(feedback.id,cmd)) return "IGNORED unencodable angle; target unchanged";
        target=value;
        return "OK target updated";
    }
    S288Command command(const Config& c) const {
        S288Command cmd;
        // 固件超时自动请求清除；反馈标志归零后恢复保护，不撤销主机使能。
        cmd.timeout_protection=!feedback.timeout;
        if(!enabled || !reference) return cmd; // 无参考时先用模式0取得一帧有效反馈。
        cmd.mode=S288Command::Mode::Foc;
        cmd.position=anchor_rotor+c.direction*(target-anchor_joint)*rad*ratio;
        cmd.kp=c.kp/(ratio*ratio);cmd.kd=c.kd/(ratio*ratio);
        return cmd; // 驱动层仍做ID、有限数、CRC和编码范围检查。
    }
};
} // namespace joint
