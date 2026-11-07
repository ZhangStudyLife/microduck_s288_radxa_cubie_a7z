# A7Z 控制服务

robotd是唯一串口所有者，robotctl是Python标准库客户端，不访问串口。
IPC使用当前用户私有目录/tmp/microduck-UID/robot.sock，Unix SOCK_SEQPACKET。
命令用空格分隔，一个消息一条请求；回复OK仅表示接受，不表示动作完成。

## 编译和启动

```bash
cd /home/microduck/control
./build.sh
./robotd /dev/ttyACM0
```

前台运行，默认向ID0~14发送模式0锁定命令。模式0不是撤力。
另开同一microduck用户终端：

```bash
cd /home/microduck/control
./robotctl status
./robotctl clear-timeout 0
./robotctl status
./robotctl lock 0
./robotctl lock-all
```

MIT调用格式（必须替换占位参数，不直接复制执行）：

```text
./robotctl mit ID POSITION VELOCITY KP KD TORQUE DURATION_MS
```

全部使用转子端物理单位：位置rad、速度rad/s、增益Nm/rad与Nm/(rad/s)、力矩Nm。
时间20~10000ms；提供绝对位置目标，非相对位移，无轨迹插值。
初始位置应从新鲜反馈获取，增益和运动范围需按机构决定；编码范围不是机械安全范围。
未收到100ms内新鲜反馈、离线、故障、警告、超时标志未清除时拒绝MIT。
清除超时使用模式0、timeout=0，下一次事务恢复timeout=1，必须查询反馈确认清除成功。
任何一次通信超时取消该台运动目标，恢复后不自动续跑。到期回模式0锁定，不是撤力。
其他电机继续轮询；公共USB或物理总线错误会结束服务，不能软件隔离电气短路。
Ctrl+C正常退出时尽力向各ID发送锁定，不保证断线设备收到；强杀不执行收尾。

## 调度

单一控制线程，20ms绝对周期；单次发送+接收预算1ms，整轮预留1ms。
在线设备先服务；三次连续超时降频到至少500ms探测，每轮至多一台离线探测。
轮换起始ID避免预算紧张时固定饿死后面的设备，错过周期不集中补发。
IPC线程只更新内存命令和读取状态，串口IO期间不持锁。
status提供反馈年龄、在线标记、错误、超时标志、统计；从未收到反馈时age_ms=-1，数值字段不是有效测量。
同一ID多条合法MIT请求采用最后一条目标，不积压。当前仅本机当前用户可调用，无网络接口。

## 边界

当前提供低层MIT调试服务，不包含步态、IMU、关节标定、限位轨迹或全身故障策略。
不自动开机启动、不自动恢复运动。模型接入与机器人的故障姿态需后续验证。
普通Linux/USB不保证硬实时，15台真实硬件50Hz仍需要验收。


## 直接指定位置

`./robotctl position 2 5`：ID 2到输出端绝对5度；默认发送1秒后回模式0锁定。
`./robotctl lock 2`：提前取消目标并锁定。到期锁定不是撤力，也不保证已经到达目标。
查看`./robotctl status`中的`position_deg`获取同一坐标系的当前位置。
该角度由转子累计位置除288.35转换而来，不是`output_degrees`单圈传感器角度，未做机械零点标定。
角度是绝对目标，不是每次追加；无轨迹插值，不要直接跳到远处目标。

固定转子端MIT参数：速度0、Kp=0.00013545、Kd=0.00000374、前馈力矩0。
来源：本项目hardware_reference/unitree_S288/research_20261023/microduck-s288/
software/training/src/mjlab_microduck/robot/microduck_constants.py中的S288 M1 BAM配方：
S288_IDENTIFIED_KP_SDK=9.03e-5乘S288_M1_BAM_KP_SCALE=1.5；Kd=3.74e-6。
这只是借用参考控制增益，没有对本机重新标定，不包含仿真的输出力矩限幅。
完整MIT接口仍可覆盖全部参数。position使用与MIT相同的反馈和故障检查。


## 终端状态显示

- `./robotctl status`：彩色紧凑表格，离线ID合并显示。
- `./robotctl status 2`：查看2号电机全部原始字段。
- `./robotctl watch`：约5Hz原地刷新；Ctrl+C退出查看，不停止电机。
- `./robotctl raw-status`：保留原来的键值输出；旧脚本解析状态时改用此命令。

绿色表示在线且错误、警告、超时标志均为0；红色表示异常；灰色表示离线或无新鲜反馈。
颜色不表示温度已通过安全判断。位置为输出端position_deg，速度为转子端rad/s。
客户端仅使用Python标准库，无需安装依赖。文件重定向自动禁用颜色。


## systemd服务（板上已安装，默认不启用开机自启）

首次安装或修改unit后执行：
```bash
sudo install -m 644 /home/microduck/control/robotd.service /etc/systemd/system/robotd.service
sudo systemctl daemon-reload
```
日常管理：
```bash
sudo systemctl start robotd
systemctl status robotd --no-pager
sudo journalctl -u robotd -n 30 --no-pager
sudo systemctl stop robotd
```
服务以microduck运行，不自动重启；关闭SSH不影响服务。
使用systemd时不要再手动启动另一个robotd，以免争用串口。
停止服务会尽力锁定电机并释放串口，不等于断电撤力。
