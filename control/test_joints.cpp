// 纯状态/IPC测试，不打开真实串口。test.sh在临时目录运行，绝不覆盖实际标定。
#define main robotd_program_main
#include "robotd.cpp"
#undef main
#include <cassert>
#include <iostream>

// 链接器只在此测试中替换fsync，证明持久化等待磁盘时没有占用控制锁。
extern "C" int __real_fsync(int);
extern "C" int __wrap_fsync(int fd) {
    bool acquired=false;
    std::thread probe([&]{acquired=mutex.try_lock();if(acquired)mutex.unlock();});probe.join();
    assert(acquired);return __real_fsync(fd);
}
void expect(const std::string& cmd, const char* prefix) {
    auto reply=request(cmd);
    if(reply.rfind(prefix,0)!=0){std::cerr<<cmd<<" -> "<<reply<<'\n';assert(false);}
}
int main() {
    auto now=Clock::now();joint::Config c;c.name="test";c.calibrated=true;c.zero=359;c.low=-40;c.high=60;
    assert(joint::wrap(1-359)==2 && joint::wrap(359-1)==-2);
    joint::State j;S288State f;f.output_degrees=9;f.rotor_position=100;
    j.observe(f,now,c);assert(j.q(c)==10);
    assert(j.arm(c,now).rfind("OK",0)==0 && j.enabled && j.reference);
    assert(j.command(c).mode==S288Command::Mode::Foc && j.command(c).position==100);
    for(double q:{2.,30.,-30.,50.,60.,-40.})assert(j.move(c,q).rfind("OK",0)==0);
    auto cmd=j.command(c);assert(std::abs(cmd.position-(100-50*joint::ratio*joint::rad))<1e-8);
    for(double q:{61.,-41.,double(NAN),double(INFINITY)})assert(j.move(c,q).rfind("IGNORED",0)==0 && j.target==-40);
    j.arm(c,now);assert(j.target==-40); // 重复使能无副作用。
    f.output_degrees=60;f.error=0x200;f.warning=1;f.case_temp=125;f.winding_temp=255;f.mode=0;
    j.observe(f,now+20ms,c);assert(j.q(c)==61 && j.enabled && j.target==-40);
    assert(j.move(c,0).rfind("OK",0)==0); // 实测越界和故障不阻止合法回程目标。
    for(int i=2;i<=2000;++i) {
        j.observe(f,now+i*20ms,c);assert(j.move(c,i%2?10:-10).rfind("OK",0)==0);
        assert(j.command(c).mode==S288Command::Mode::Foc);
    }
    // 丢一帧或主机调度延迟不重设参考；三次超时的disconnect由调度层调用。
    auto old=j.target;assert(j.command(c).mode==S288Command::Mode::Foc && j.target==old);
    j.disconnect();assert(j.enabled && !j.reference && j.target==0);
    assert(j.command(c).mode==S288Command::Mode::Lock);
    assert(j.move(c,2).rfind("ERR",0)==0);
    f={};f.output_degrees=19;f.rotor_position=500;j.observe(f,now+41s,c);
    assert(j.enabled && j.target==20 && j.command(c).position==500); // 不重放旧目标。
    j.move(c,30);f.timeout=1;f.rotor_position=600;j.observe(f,now+42s,c);
    assert(j.target==20 && j.command(c).position==600 && !j.command(c).timeout_protection);
    j.move(c,25);j.observe(f,now+43s,c);assert(j.target==25); // 持续置位不重复捕获。
    f.timeout=0;j.observe(f,now+44s,c);assert(j.command(c).timeout_protection);
    f.error=0x4000;f.rotor_position=700;j.observe(f,now+45s,c);assert(j.command(c).position==700 && j.target==20);
    j.move(c,30);j.observe(f,now+46s,c);assert(j.target==30);
    j.stop();assert(!j.enabled && !j.reference && j.command(c).mode==S288Command::Mode::Lock);
    j.observe(f,now+47s,c);assert(!j.enabled); // stop后恢复反馈不自动使能。
    joint::State pending;pending.arm(c,now);assert(pending.enabled && !pending.reference);
    pending.observe(f,now,c);assert(pending.reference);
    auto uncal=c;uncal.calibrated=false;joint::State u;u.observe(f,now,uncal);u.arm(uncal,now);
    assert(u.command(uncal).mode==S288Command::Mode::Foc && u.move(uncal,0).rfind("ERR",0)==0);
    auto invalid=c;invalid.kp=2129;assert(!joint::valid(invalid));invalid.kp=0;invalid.kd=0;assert(joint::valid(invalid));
    invalid.kd=21.29;assert(!joint::valid(invalid));

    configs=joint::defaults();motors={};config_ok=true;
    auto sample=[&](double sensor,double rotor=100) {
        S288State state;state.output_degrees=sensor;state.rotor_position=rotor;
        motors[0].joint.observe(state,Clock::now(),configs[0]);
    };
    auto& m=motors[0];
    expect("position 0 10","ERR");expect("mit 0 0 0 0 0 0 1000","ERR");
    sample(359);expect("joint arm 0","OK");assert(m.joint.enabled);
    expect("joint calibrate 0 begin","OK");assert(!m.joint.enabled);
    m.joint.good=Clock::now()-1s;expect("joint calibrate 0 zero","ERR");assert(m.joint.stage==1);
    sample(359);expect("joint calibrate 0 zero","OK");
    expect("joint calibrate 0 lower","ERR");expect("joint calibrate 0 upper","ERR");
    sample(29);expect("joint calibrate 0 upper","OK");
    sample(9);expect("joint calibrate 0 lower","ERR");
    sample(349);expect("joint calibrate 0 lower","OK");
    sample(5);expect("joint calibrate 0 center","ERR");assert(m.joint.stage==4);
    sample(4);expect("joint calibrate 0 center","OK"); // +5°，自动保存。
    assert(configs[0].calibrated && configs[0].zero==359 && configs[0].low==-10 && configs[0].high==30);
    assert(!m.joint.enabled && m.joint.stage==0);
    auto disk=joint::defaults();assert(joint::load(config_path,disk) && disk[0].zero==359);
    sample(9);expect("joint arm 0","OK");expect("joint move 0 2","OK");expect("joint move 0 30","OK");
    for(const char* value:{"31","-11","nan","inf","nonsense","1oops","1e999"})
        expect(std::string("joint move 0 ")+value,"IGNORED");
    assert(m.joint.enabled && m.joint.target==30);
    expect("joint arm 0","OK");assert(m.joint.target==30);
    expect("joint params 0 2 .03","OK");assert(configs[0].kp==2 && m.joint.enabled && m.joint.target==30);
    assert(std::abs(m.joint.command(configs[0]).kp-2/(joint::ratio*joint::ratio))<1e-15);
    assert(joint::load(config_path,disk) && disk[0].kp==2);
    expect("joint params 0","OK kp=2");expect("joint params 0 -1 .03","ERR");expect("joint params 0 nan .03","ERR");
    assert(configs[0].kp==2);
    // 制造rename失败，检查参数/标定保存失败均不改内存和原文件。
    assert(rename("joints.conf","original.conf")==0 && mkdir("joints.conf",0700)==0);
    expect("joint params 0 3 .04","ERR");assert(configs[0].kp==2);
    sample(100);expect("joint calibrate 0 begin","OK");expect("joint calibrate 0 zero","OK");
    sample(60);expect("joint calibrate 0 upper","OK");sample(130);expect("joint calibrate 0 lower","OK");
    sample(106);expect("joint calibrate 0 center","ERR"); // 反向安装-6°不通过。
    sample(105);expect("joint calibrate 0 center","ERR"); // -5°通过采样，但写盘失败。
    assert(m.joint.stage==4 && configs[0].zero==359 && !m.joint.enabled);
    assert(rmdir("joints.conf")==0 && rename("original.conf","joints.conf")==0);
    expect("joint calibrate 0 center","OK");assert(configs[0].zero==100 && configs[0].direction==-1);
    assert(configs[0].low==-30 && configs[0].high==40 && !m.joint.enabled);
    sample(100);expect("joint calibrate 0 begin","OK");expect("joint calibrate 0 zero","OK");
    expect("joint calibrate 0 cancel","OK");assert(configs[0].zero==100 && m.joint.stage==0);
    sample(120);expect("joint arm 0","OK");expect("clear-timeout 0","OK");assert(m.joint.enabled);
    expect("joint stop 0","OK");expect("joint move 0 0","ERR");
    assert(joint::load(config_path,disk));joint::State restarted;assert(!restarted.enabled);
    assert(request("status").find("enabled=0")!=std::string::npos);
    config_ok=false;expect("joint arm 0","ERR");expect("joint stop 0","OK");
    std::cout<<"PASS: persistent enable, latest target, ignored inputs, reconnect/reset, calibration retry, params, atomic save and unlocked disk IO\n";
}
