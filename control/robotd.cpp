#include "../s288/s288.hpp"
#include "joints.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
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
    joint::State joint;
    Clock::time_point probe{};
    unsigned misses=0, failures=0, successes=0, foc_sent=0, last_mode=0;
    long long max_us=0;
    bool offline=false, clear=false;
};
std::array<Motor,15> motors;
std::mutex mutex;
unsigned long rounds=0, overruns=0;
bool bus_fault=false, config_ok=false;
std::array<joint::Config,15> configs=joint::defaults();
const std::string config_path="joints.conf";

bool number(const std::string& text, double& value) {
    char* end=nullptr;errno=0;value=std::strtod(text.c_str(),&end);
    return !text.empty() && end==text.c_str()+text.size() && errno!=ERANGE && std::isfinite(value);
}
// 只有一个IPC线程修改配置。文件写入期间释放锁，让50Hz控制继续使用旧配置；
// 原子保存成功后再更新内存。保存失败不改变正在使用的参数。
bool persist(const std::array<joint::Config,15>& candidate, std::unique_lock<std::mutex>& guard) {
    guard.unlock();bool ok=joint::save(config_path,candidate);guard.lock();
    if(ok) configs=candidate;
    return ok;
}
std::string joint_request(std::istringstream& in, std::unique_lock<std::mutex>& guard) {
    std::string op,name,step,extra;in>>op>>name;
    unsigned id=15;
    for(unsigned i=0;i<15;++i) if(name==configs[i].name || name==std::to_string(i)) id=i;
    if(id==15) return "ERR unknown joint";
    auto& m=motors[id];auto& j=m.joint;auto& c=configs[id];auto now=Clock::now();
    if(op=="stop") {
        if(in>>extra) return "ERR unexpected arguments";
        j.stop();m.clear=false;return "OK disabled; mode0 lock, not torque-off";
    }
    if(!config_ok) return "ERR invalid joints.conf";
    if(op=="arm") {if(in>>extra)return "ERR unexpected arguments";return j.arm(c,now);}
    if(op=="home" || op=="move") {
        double value=0;
        if((op=="move" && (!(in>>extra) || !number(extra,value))) || (in>>extra))
            return "IGNORED invalid angle; target unchanged";
        return j.move(c,value);
    }
    if(op=="params") {
        if(!(in>>step)) {
            std::ostringstream out;out<<"OK kp="<<c.kp<<" kd="<<c.kd<<" kp_unit=Nm/rad kd_unit=Nm/(rad/s)";return out.str();
        }
        auto candidate=configs;
        if(!number(step,candidate[id].kp) || !(in>>step) || !number(step,candidate[id].kd) || (in>>extra))
            return "ERR expected finite KP KD";
        if(!joint::valid(candidate[id])) return "ERR KP/KD must be nonnegative and fit S288 protocol range";
        if(!persist(candidate,guard)) return "ERR saving parameters failed; previous values retained";
        // 标定期间也允许调参，避免稍后保存标定把新增益覆盖回旧值。
        if(j.stage) {j.draft.kp=c.kp;j.draft.kd=c.kd;}
        return "OK parameters saved and applied";
    }
    if(op!="calibrate" || !(in>>step) || (in>>extra)) return "ERR unknown joint operation or invalid arguments";
    if(step=="cancel") {j.stop();return "OK calibration cancelled";}
    if(step=="begin") {
        j.stop();j.draft=c;j.draft.low=-1;j.draft.high=1;j.stage=1;
        return "OK calibration started; manual poses only; mode0 is lock, not torque-off";
    }
    if(!j.fresh(now)) return "ERR no fresh valid feedback; retry this sample";
    if(step=="zero" && j.stage==1) {j.draft.zero=j.feedback.output_degrees;j.stage=2;}
    else if((step=="upper" && j.stage==2) || (step=="lower" && j.stage==3)) {
        double delta=joint::wrap(j.feedback.output_degrees-j.draft.zero);
        if(delta==0) return "ERR endpoint must differ from zero; retry this sample";
        auto candidate=j.draft;
        if(step=="upper") {candidate.direction=delta>0?1:-1;candidate.high=std::abs(delta);}
        else candidate.low=candidate.direction*delta;
        if((step=="upper" && candidate.high>=180) || (step=="lower" && !joint::valid(candidate)))
            return "ERR endpoints must surround zero and span less than 180 degrees";
        j.draft=candidate;++j.stage;
    } else if(step=="center" && j.stage==4) {
        double delta=j.q(j.draft);
        if(std::abs(delta)>5) {
            std::ostringstream out;out<<"ERR center deviation="<<delta<<" degrees; return within +/-5 and retry";return out.str();
        }
        auto candidate=configs;candidate[id]=j.draft;candidate[id].calibrated=true;
        if(!persist(candidate,guard)) return "ERR saving calibration failed; previous values retained; retry center";
        j.stop();
        std::ostringstream out;out<<"OK calibration saved zero_sensor_deg="<<c.zero<<" direction="<<c.direction
            <<" min_deg="<<c.low<<" max_deg="<<c.high;return out.str();
    } else return "ERR order: zero -> upper -> lower -> center (auto-save)";
    std::ostringstream out;out<<"OK stage="<<j.stage<<" zero_sensor_deg="<<j.draft.zero
        <<" direction="<<j.draft.direction<<" min_deg="<<j.draft.low<<" max_deg="<<j.draft.high;return out.str();
}
// IPC只更新内存目标，不打开串口。enabled是用户意图，rx_mode/error是电机报告，两者分开显示。
std::string request(const std::string& text) {
    std::istringstream in(text);std::string op,extra;in>>op;
    std::unique_lock<std::mutex> guard(mutex);
    if(op=="position" || op=="mit") return "ERR raw motion disabled; use joint move";
    if(op=="joint") return joint_request(in,guard);
    if(op=="status") {
        if(in>>extra) return "ERR unexpected arguments";
        auto now=Clock::now();std::ostringstream out;
        out<<"rounds="<<rounds<<" overruns="<<overruns<<" config_ok="<<config_ok<<" bus_fault="<<bus_fault<<'\n';
        for(unsigned id=0;id<15;++id) {
            const auto& m=motors[id];const auto& j=m.joint;const auto& f=j.feedback;const auto& c=configs[id];
            auto age=j.good==Clock::time_point{}?-1:std::chrono::duration_cast<std::chrono::milliseconds>(now-j.good).count();
            out<<"id="<<id<<" name="<<c.name<<" online="<<(!bus_fault && !m.offline && j.fresh(now))
                <<" enabled="<<j.enabled<<" reference="<<j.reference<<" calibrating="<<j.stage
                <<" calibrated="<<c.calibrated<<" age_ms="<<age<<" joint_deg="<<(c.calibrated?j.q(c):0)
                <<" target_deg="<<j.target<<" min_deg="<<c.low<<" max_deg="<<c.high
                <<" zero_sensor_deg="<<c.zero<<" direction="<<c.direction<<" kp="<<c.kp<<" kd="<<c.kd
                <<" rx_mode="<<f.mode<<" tx_mode="<<m.last_mode<<" timeout_flag="<<f.timeout
                <<" error="<<f.error<<" warning="<<f.warning<<" case_temp="<<f.case_temp<<" winding_temp="<<f.winding_temp
                <<" position="<<f.rotor_position<<" position_deg="<<f.rotor_position/joint::ratio/joint::rad
                <<" output_degrees="<<f.output_degrees<<" speed="<<f.rotor_speed<<" torque="<<f.rotor_torque
                <<" voltage="<<f.voltage<<" ok="<<m.successes<<" failures="<<m.failures
                <<" max_us="<<m.max_us<<" foc_sent="<<m.foc_sent<<'\n';
        }
        return out.str();
    }
    if(op=="lock-all") {
        if(in>>extra) return "ERR unexpected arguments";
        for(auto& m:motors){m.joint.stop();m.clear=false;}
        return "OK all disabled; mode0 lock";
    }
    unsigned id;
    if(!(in>>id) || id>=15 || (in>>extra)) return "ERR expected ID 0..14";
    if(op=="lock") {motors[id].joint.stop();motors[id].clear=false;return "OK disabled; mode0 lock";}
    if(op=="clear-timeout") {motors[id].clear=true;return "OK clear requested; enable intent unchanged";}
    return "ERR unknown command";
}
void serve(int listener) {
    while(running) {
        pollfd p{listener,POLLIN,0};if(poll(&p,1,100)<=0)continue;
        int client=accept4(listener,nullptr,nullptr,SOCK_CLOEXEC|SOCK_NONBLOCK);if(client<0)continue;
        pollfd q{client,POLLIN,0};
        if(poll(&q,1,100)>0) {
            char data[1024];ssize_t n=recv(client,data,sizeof(data),MSG_TRUNC);
            std::string reply=n>0 && n<ssize_t(sizeof(data))?request(std::string(data,size_t(n))):"ERR invalid request size";
            send(client,reply.data(),reply.size(),MSG_NOSIGNAL);
        }
        close(client);
    }
}
// 串口只由控制线程打开和关闭。重连时重新获取文件锁及TTY独占，不能绕开其他所有者。
void close_serial(int& owner, S288Bus& bus) {
    if(owner>=0)ioctl(owner,TIOCNXCL);
    bus.close();if(owner>=0)close(owner);owner=-1;
}
bool open_serial(const char* path, int& owner, S288Bus& bus) {
    int fd=open(path,O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);if(fd<0)return false;
    if(flock(fd,LOCK_EX|LOCK_NB)<0 || !bus.open(path) || ioctl(fd,TIOCEXCL)<0) {
        bus.close();close(fd);return false;
    }
    owner=fd;return true;
}
int main(int argc,char** argv) {
    if(argc!=2){puts("usage: robotd /dev/ttyACM0");return 1;}
    for(unsigned id=0;id<15;++id)motors[id].joint.feedback.id=id;
    config_ok=joint::load(config_path,configs);
    if(!config_ok)std::fprintf(stderr,"invalid joints.conf: arm/move disabled\n");
    umask(0077);
    std::string path="/tmp/microduck-"+std::to_string(getuid());
    if(mkdir(path.c_str(),0700)<0 && errno!=EEXIST){perror("IPC directory");return 1;}
    struct stat st{};
    if(lstat(path.c_str(),&st)<0 || !S_ISDIR(st.st_mode) || st.st_uid!=getuid() || (st.st_mode&0077))return 1;
    int lock=open((path+"/owner.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)<0){puts("robotd already running");return 1;}
    path+="/robot.sock";int listener=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);
    sockaddr_un address{};address.sun_family=AF_UNIX;std::snprintf(address.sun_path,sizeof(address.sun_path),"%s",path.c_str());
    unlink(path.c_str());
    if(bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 || listen(listener,8)<0){perror("IPC listen");return 1;}
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);std::thread ipc(serve,listener);
    S288Bus bus;int owner=-1;auto retry=Clock::now(),next=retry;
    puts("robotd ready: 15 IDs / 50Hz; persistent enable intent; startup disabled");fflush(stdout);
    while(running) {
        if(owner<0 && Clock::now()>=retry) {
            bool opened=open_serial(argv[1],owner,bus);retry=Clock::now()+500ms;
            std::lock_guard<std::mutex> guard(mutex);bus_fault=!opened;
            if(opened) {
                for(auto& m:motors){m.joint.disconnect();m.misses=0;m.offline=false;}
                std::fprintf(stderr,"serial connected; enabled joints adopt current pose\n");
            }
        }
        auto deadline=next+20ms;bool probed=false;
        for(int pass=0;pass<2 && running && owner>=0;++pass) {
            for(unsigned slot=0;slot<15 && running;++slot) {
                unsigned id=(slot+unsigned(rounds%15))%15;
                // 一次事务最多约1ms。持锁覆盖收发，避免stop已返回后仍发出旧FOC快照。
                // 配置文件写入不持此锁，因此不会让磁盘I/O阻塞控制循环。
                std::lock_guard<std::mutex> guard(mutex);
                auto& m=motors[id];auto now=Clock::now();
                if(m.offline!=(pass==1))continue;
                if(m.offline) {
                    if(probed || now<m.probe)continue;
                    probed=true;m.probe=now+500ms;
                }
                auto budget=std::min(1000us,std::chrono::duration_cast<std::chrono::microseconds>(deadline-now-1ms));
                if(budget<=0us)continue;
                auto cmd=m.joint.command(configs[id]);if(m.clear){cmd.timeout_protection=false;m.clear=false;}
                S288State f;auto result=bus.exchange(id,cmd,f,budget);auto end=Clock::now();
                m.last_mode=unsigned(cmd.mode);if(cmd.mode==S288Command::Mode::Foc)++m.foc_sent;
                m.max_us=std::max(m.max_us,static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(end-now).count()));
                if(result==S288Bus::Result::Ok) {
                    m.misses=0;m.offline=false;++m.successes;m.joint.observe(f,end,configs[id]);
                } else if(result==S288Bus::Result::Timeout) {
                    ++m.failures;
                    if(++m.misses>=3){m.offline=true;m.probe=end+500ms;m.joint.disconnect();}
                } else if(result==S288Bus::Result::Error) {
                    bus_fault=true;
                    for(auto& motor:motors){motor.joint.disconnect();motor.offline=true;}
                    close_serial(owner,bus);retry=end+500ms;
                    std::fprintf(stderr,"serial unavailable; retry in 500ms; enable intent retained\n");break;
                } // Interrupted由信号处理；InvalidArgument不发送，均不撤销使能意图。
            }
        }
        {std::lock_guard<std::mutex> guard(mutex);++rounds;next+=20ms;
            if(Clock::now()>next){++overruns;do{next+=20ms;}while(next<Clock::now());}}
        if(running)std::this_thread::sleep_until(next);
    }
    ipc.join();close(listener);unlink(path.c_str());
    if(owner>=0)for(unsigned id=0;id<15;++id){S288State f;bus.lock_and_read(id,f);}
    close_serial(owner,bus);close(lock);return 0;
}
