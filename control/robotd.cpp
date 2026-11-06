#include "../s288/s288.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <thread>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
struct Motor {
    S288State state{};
    S288Command target{};
    Clock::time_point good{}, expires{}, probe{};
    unsigned misses = 0, failures = 0, successes = 0;
    long long max_us = 0;
    bool offline = false, clear = false;
    unsigned foc_sent = 0;
    unsigned last_mode = 0;
};
std::array<Motor,15> motors;
std::mutex mutex;
unsigned long rounds = 0, overruns = 0;
bool bus_fault = false;

// IPC线程只操作内存目标，绝不打开串口。一个请求对应一份响应。
// MIT使用转子端单位；请求接受仅表示排队，不表示电机已完成动作。
std::string request(const std::string& text)
{
    std::istringstream in(text);
    std::string op, extra;
    in >> op;
    std::lock_guard<std::mutex> guard(mutex);
    auto now = Clock::now();
    if (op == "status") {
        if (in >> extra) return "ERR unexpected arguments";
        std::ostringstream out;
        out << "rounds=" << rounds << " overruns=" << overruns << " bus_fault=" << bus_fault << '\n';
        for (unsigned id=0; id<15; ++id) {
            auto& m=motors[id];
            auto age=m.good==Clock::time_point{} ? -1 : std::chrono::duration_cast<std::chrono::milliseconds>(now-m.good).count();
            out << "id=" << id << " online=" << (!m.offline && age>=0 && age<100)
                << " age_ms=" << age << " active=" << (m.expires>now)
                << " clear_pending=" << m.clear << " ok=" << m.successes
                << " failures=" << m.failures << " max_us=" << m.max_us
                << " rx_mode=" << m.state.mode << " tx_mode=" << m.last_mode << " foc_sent=" << m.foc_sent
                << " position_deg=" << m.state.rotor_position / 288.35 * 180.0 / 3.141592653589793
                << " output_degrees=" << m.state.output_degrees
                << " winding_temp=" << m.state.winding_temp
                << " timeout_flag=" << m.state.timeout << " error=" << m.state.error
                << " warning=" << m.state.warning << " position=" << m.state.rotor_position
                << " speed=" << m.state.rotor_speed << " torque=" << m.state.rotor_torque
                << " voltage=" << m.state.voltage << " case_temp=" << m.state.case_temp << '\n';
        }
        return out.str();
    }
    if (op == "lock-all") {
        if (in >> extra) return "ERR unexpected arguments";
        for (auto& m:motors) { m.expires={}; m.clear=false; }
        return "OK lock queued for all motors";
    }
    unsigned id;
    if (!(in>>id) || id>=15) return "ERR id must be 0..14";
    auto& m=motors[id];
    if (op=="lock") {
        if (in>>extra) return "ERR unexpected arguments";
        m.expires={}; m.clear=false;
        return "OK lock queued";
    }
    if (bus_fault || m.offline || m.good==Clock::time_point{} || now-m.good>100ms)
        return "ERR motor feedback unavailable or stale";
    if (op=="clear-timeout") {
        if (in>>extra) return "ERR unexpected arguments";
        m.expires={}; m.clear=true;
        return "OK clear queued; inspect timeout_flag, protection restored on next exchange";
    }
    if (op=="mit" || op=="position") {
        S288Command cmd;
        cmd.mode=S288Command::Mode::Foc;
        int duration=1000;
        if (op=="position") {
            // 输出端绝对角度（度），零点来自转子累计位置，不是单圈OutPos传感器。
            // S288 M1 BAM参考配方：Kp=9.03e-5*1.5，Kd=3.74e-6，均为转子端。
            // 只借用控制增益；不包含仿真模型的摩擦补偿或力矩限幅。
            if (!(in>>cmd.position) || (in>>extra))
                return "ERR usage: position ID DEGREES";
            cmd.position *= 3.141592653589793 / 180.0 * 288.35;
            cmd.kp=0.00013545;
            cmd.kd=0.00000374;
            // speed和torque保持默认0；服务按50Hz发送，1秒后恢复锁定。
        } else {
            // 完整MIT顺序：id position velocity kp kd torque duration_ms，转子端单位。
            if (!(in>>cmd.position>>cmd.speed>>cmd.kp>>cmd.kd>>cmd.torque>>duration) || (in>>extra))
                return "ERR invalid MIT arguments";
        }
        if (duration<20 || duration>10000 || !S288Bus::valid_command(id,cmd))
            return "ERR invalid MIT values or duration (20..10000 ms)";
        if (m.clear || m.state.timeout || m.state.error || m.state.warning)
            return "ERR clear motor fault/timeout before MIT";
        m.target=cmd; m.expires=now+std::chrono::milliseconds(duration);
        return "OK MIT queued; expiry returns to mode0 lock, not torque-off";
    }
    return "ERR unknown command";
}

void serve(int listener)
{
    while (running) {
        pollfd p{listener,POLLIN,0};
        if (poll(&p,1,100)<=0) continue;
        int client=accept4(listener,nullptr,nullptr,SOCK_CLOEXEC|SOCK_NONBLOCK);
        if (client<0) continue;
        // 慢客户端只占IPC线程至多100ms，不阻塞50Hz控制线程。
        pollfd q{client,POLLIN,0};
        if (poll(&q,1,100)>0) {
            char data[1024];
            ssize_t n=recv(client,data,sizeof(data),MSG_TRUNC);
            std::string reply=n>0 && n<ssize_t(sizeof(data)) ? request(std::string(data,size_t(n))) : "ERR invalid request size";
            send(client,reply.data(),reply.size(),MSG_NOSIGNAL);
        }
        close(client);
    }
}

int main(int argc,char** argv)
{
    if (argc!=2) { puts("usage: robotd /dev/ttyACM0 (default sends mode0 lock)"); return 1; }
    umask(0077);
    // IPC仅限当前用户；锁定真实设备文件，别名路径仍对应同一个设备节点。
    int owner=open(argv[1],O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (owner<0 || flock(owner,LOCK_EX|LOCK_NB)<0) { perror("serial ownership"); return 1; }
    S288Bus bus;
    if (!bus.open(argv[1]) || ioctl(owner,TIOCEXCL)<0) { perror("serial open/exclusive"); return 1; }
    // 同一用户只有一个服务端点；锁防止两个实例删除彼此socket。
    std::string path="/tmp/microduck-"+std::to_string(getuid());
    if (mkdir(path.c_str(),0700)<0 && errno!=EEXIST) { perror("runtime directory"); return 1; }
    struct stat st{};
    if (lstat(path.c_str(),&st)<0 || !S_ISDIR(st.st_mode) || st.st_uid!=getuid() || (st.st_mode&0077)) {
        puts("unsafe runtime directory"); return 1;
    }
    int lock=open((path+"/owner.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if (lock<0 || flock(lock,LOCK_EX|LOCK_NB)<0) { puts("robotd already running"); return 1; }
    path+="/robot.sock";
    int listener=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);
    sockaddr_un address{}; address.sun_family=AF_UNIX;
    std::snprintf(address.sun_path,sizeof(address.sun_path),"%s",path.c_str());
    unlink(path.c_str());
    if (bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 || listen(listener,8)<0) {
        perror("IPC listen"); return 1;
    }
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
    std::thread ipc(serve,listener);
    auto next=Clock::now();
    puts("robotd ready: 15 IDs / 50Hz; default LOCK, MIT requires explicit duration");
    while (running) {
        auto deadline=next+20ms;
        bool probed=false;
        for (int pass=0;pass<2 && running;++pass) {
        for (unsigned slot=0;slot<15 && running;++slot) {
            unsigned id=(slot+unsigned(rounds%15))%15;
            S288Command cmd;
            bool clear=false;
            {
                std::lock_guard<std::mutex> guard(mutex);
                auto& m=motors[id]; auto now=Clock::now();
                if (m.offline != (pass==1)) continue;
                if (m.offline) {
                    if (probed || now<m.probe) continue;
                    probed=true; m.probe=now+500ms;
                }
                if (now<m.expires) cmd=m.target;
                clear=m.clear; m.clear=false;
                if (clear) cmd.timeout_protection=false;
            }
            auto start=Clock::now();
            auto budget=std::min(1000us,std::chrono::duration_cast<std::chrono::microseconds>(deadline-start-1ms));
            if (budget<=0us) {
                if (clear) { std::lock_guard<std::mutex> guard(mutex); motors[id].clear=true; }
                continue;
            }
            S288State feedback;
            auto result=bus.exchange(id,cmd,feedback,budget);
            auto end=Clock::now();
            std::lock_guard<std::mutex> guard(mutex);
            auto& m=motors[id];
            m.last_mode=unsigned(cmd.mode);
            if (cmd.mode==S288Command::Mode::Foc) ++m.foc_sent;
            m.max_us=std::max(m.max_us,static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(end-start).count()));
            if (result==S288Bus::Result::Ok) {
                m.state=feedback; m.good=end; m.misses=0; m.offline=false; ++m.successes;
                if (feedback.error || feedback.warning || feedback.timeout) m.expires={};
            } else if (result==S288Bus::Result::Timeout) {
                ++m.failures; m.expires={}; // 一次丢帧即撤销运动目标，不自动恢复。
                if (++m.misses>=3) { m.offline=true; m.probe=end+500ms; }
            } else {
                if (result!=S288Bus::Result::Interrupted) bus_fault=true;
                running=0;
            }
        }
        }
        {
            std::lock_guard<std::mutex> guard(mutex);
            ++rounds; next+=20ms;
            if (Clock::now()>next) { ++overruns; do { next+=20ms; } while (next<Clock::now()); }
        }
        if (running) std::this_thread::sleep_until(next);
    }
    ipc.join(); close(listener); unlink(path.c_str());
    // 正常退出尽力向所有ID发送锁定；不承诺断线设备已收到，也不是撤力。
    for (unsigned id=0;id<15;++id) { S288State state; bus.lock_and_read(id,state); }
    ioctl(owner,TIOCNXCL); bus.close(); close(owner); close(lock);
    return bus_fault ? 1 : 0;
}
