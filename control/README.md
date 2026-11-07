# S288 持续位置控制

`robotd`独占串口，以20ms周期轮询15个ID；`robotctl`通过Unix Socket更新目标和查询反馈。
正常控制只需理解：标定映射、使能意图、一个最新目标、输出端Kp/Kd。
不排队、不等到达、不插值、不自动到期。新目标直接覆盖旧目标。

## 日常使用

```bash
cd ~/control
./robotctl joints
./robotctl joint params right_ankle       # 查询输出端Kp、Kd
./robotctl joint arm right_ankle          # 开始持续FOC，先保持当前位置
./robotctl joint move right_ankle 2       # 绝对机械角度，不是追加2度
./robotctl joint move right_ankle -2      # 覆盖上一个目标，不必等它到达
./robotctl joint home right_ankle         # 等价于move right_ankle 0
./robotctl joint stop right_ankle         # 撤销使能，回模式0锁定
./robotctl watch                         # Ctrl+C只退出查看
```

名称均可换成ID，例如`joint move 0 2`。角度必须在该关节保存的范围内，上下限本身允许。
未标定的电机可以`arm`保持当前转子位置，但必须标定后才能使用机械角度`move`。
已使能时重复`arm`不改变目标；未使能时`move`提示先使能。
没有反馈时`arm`记住使能意图，等收到反馈后保持当前位置，期间不缓存机械目标。

范围外、NaN、无穷大或无法解析的角度返回`IGNORED`，退出码0；旧目标和使能不变。
例如范围为[-40,60]时，`move 0 61`被忽略，`move 0 60`合法。
原始`mit/position`入口仍禁用，直接IPC请求也不能绕过机械目标接口。

## 标定：四次采样

```bash
./robotctl joint calibrate               # 列出电机，输入名称或ID
# 或直接选中
./robotctl joint calibrate right_ankle
```

进入后取消该电机的持续FOC，继续用模式0查询。依次手动摆位并按回车：

1. 机械零点：保存输出端单圈编码器角度。
2. 远端：该方向定义为关节正方向，保存正边界。
3. 近端：保存负边界。
4. 回到第一次的零点：误差在±5°内便自动保存，不重新定义零点。

只有新鲜（100ms以内）、通过驱动CRC和ID校验的反馈才可采样。没有有效数据时重采当前点。
两端须在零点两侧，总跨度小于180°。跨360°用有符号角差处理，例如359°到1°是+2°。
回中失败只重试回中；取消或写盘失败不改变旧标定。保存后仍未使能，下一次直接`arm`，无需`reference`。
不再要求BEGIN/SAVE/ZERO确认词、稳定窗口或20°余量；保存的就是实际采集的边界。

此过程不自动碰端点、不自动回中。模式0是锁定，不是撤力，不能强行反驱锁定机构。
输入限幅只约束目标，不能保证实际位置没有超调；采点时需要自行选择机构可接受的边界。

## 坐标和参数

```text
机械角 = direction × wrap180(输出端单圈角 − 保存的零位)
转子目标 = 参考转子位置
         + direction × 288.35 × (目标机械角 − 参考机械角) × π/180
```

建立参考时，两种角度取自同一帧。参考固定使用，直到重新使能、掉线恢复、超时标志上升沿
或异常复位位（MError bit14）上升沿；不逐帧用传感器误差重设参考。

```bash
./robotctl joint params right_ankle
./robotctl joint params right_ankle 1 0.02
```

参数依次为输出端Kp（N·m/rad）、Kd（N·m/(rad/s)）。除以288.35²后编码为转子增益；
前馈力矩、期望速度始终为0。Kp、Kd须非负、有限且符合协议16位有符号整数的非负编码范围：

```text
Kp ≤ 32767 × 288.35² / 1280000
Kd ≤ 32767 × 288.35² / 128000000
```

这是可编码范围，不是推荐调参值。协议定点量化会截断很小的参数。
设置先写临时文件并原子替换，成功后才更新内存，下一控制周期生效；失败保留原值。
磁盘写入不占用控制循环锁。沿用九列`joints.conf`：

```text
id name calibrated zero_sensor_deg direction min_deg max_deg kp kd
```

使能和目标不写入文件。启动加载配置，配置错误时仍可查询，但拒绝使能；不要运行中手改文件期待热加载。

## 丢帧、重连和电机报告

- 单次收发超时不改变目标或使能；连续三次事务超时标记离线，以500ms间隔低频探测。
- 离线保留使能意图，但清除目标及参考。恢复后保持新当前位置，等待新目标，不重放旧动作。
- USB I/O错误不退出服务，每500ms尝试重新打开原设备路径并获取串口独占。
- 超时反馈置位时自动发送清除位；反馈清零后恢复协议超时保护位。
- 实测越界、模式差异、温度、错误码、警告和调度延迟只显示，不自动取消主机使能。
- `stop`/`lock`/`lock-all`主动停用；进入标定也停用选中电机。服务退出收尾发送模式0，重启默认未使能。

软件保证的是使能意图与持续发送，不能强迫固件在过温、过流等硬件保护期间输出力矩。
模式0锁定不等于撤力或硬件急停。仍保留CRC、ID、数值编码范围和每次串口事务时间预算。
协议没有事务序号，不能完全排除迟到的同ID反馈被当作当前回复。

`status/watch`分别显示主机使能、反馈模式、机械角、目标和温度；电机报错时不再把主机显示成已停用。
`joint status NAME`显示详细字段，`raw-status`输出机器可读键值。离线反馈可能是旧值。
`reference=0`表示还未建立当前换算参考，`calibrating=1..4`仅表示采样步骤。
`position`是转子弧度，`position_deg`是转子换算角，`output_degrees`是输出端单圈角；
控制输入应使用`joint_deg`对应的机械坐标。未标定时界面以`--`隐藏机械角和目标。

## 编译、测试和服务

```bash
cd ~/control
./test.sh             # UBSan状态/IPC/保存测试，不访问真实串口
python3 test_bus.py    # 虚拟串口测试，独立IPC和临时配置，不控制实物
sudo systemctl stop robotd
./build.sh
sudo systemctl start robotd
./robotctl status
```

只有确认无位置控制或标定时才停服替换。部署前备份程序和真实`joints.conf`；不要用仓库模板覆盖实物标定。
systemd使用`robotd.service`中的WorkingDirectory查找配置：

```bash
sudo install -m 644 robotd.service /etc/systemd/system/robotd.service
sudo systemctl daemon-reload
sudo systemctl start robotd
systemctl status robotd --no-pager
journalctl -u robotd -n 30 --no-pager
```

测试覆盖目标覆盖、非法目标、使能幂等、故障反馈、跨圈/反装标定、回中重试、取消、保存失败、
参数持久化，以及虚拟串口的丢帧/掉线/USB重开。模拟测试不能替代实物超调、负载和机械间隙验收。
