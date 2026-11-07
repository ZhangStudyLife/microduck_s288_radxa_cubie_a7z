#!/usr/bin/env python3
"""用PTY模拟15台电机，不访问真实USB；测试真实串口驱动、调度和IPC。"""
import math
import os
from pathlib import Path
import pty
import select
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time


def crc(data):
    value = 0xffffffff
    for word, in struct.iter_unpack('<I', data):
        for bit in range(31, -1, -1):
            top = (value >> 31) ^ ((word >> bit) & 1)
            value = (value << 1) & 0xffffffff
            if top:
                value ^= 0x04c11db7
    return value


with tempfile.TemporaryDirectory(prefix='s288-bus-') as directory:
    root = Path(directory)
    source = Path(__file__).resolve().parent
    (root / 'control').mkdir()
    (root / 's288').symlink_to(source.parent / 's288')
    (root / 'control/joints.hpp').symlink_to(source / 'joints.hpp')
    # 仅给测试副本改IPC目录，避免干扰板上正在运行的真实robotd。
    text = (source / 'robotd.cpp').read_text().replace('"/tmp/microduck-"', '"' + str(root / 'ipc-') + '"')
    (root / 'control/robotd.cpp').write_text(text)
    subprocess.run(['g++', '-O2', '-std=c++17', '-pthread', str(root / 'control/robotd.cpp'),
                    str(root / 's288/s288.cpp'), '-o', str(root / 'robotd')], check=True)
    (root / 'joints.conf').write_text('\n'.join(
        f'{i} {"right_ankle" if i == 0 else "motor_" + str(i)} {int(i == 0)} 0 1 -40 60 1 .02'
        for i in range(15)) + '\n')
    endpoint = root / f'ipc-{os.getuid()}/robot.sock'
    cli = (source / 'robotctl').read_text().replace('/tmp/microduck-%d/robot.sock', str(root / 'ipc-%d/robot.sock'))
    (root / 'robotctl').write_text(cli)
    device = root / 'serial'
    state = dict(angle=10, rotor=100, drop=0, timeout=0, error=0, temperature=30)
    frames = []
    shutdown = threading.Event()

    def new_serial():
        master, slave = pty.openpty()
        device.unlink(missing_ok=True)
        device.symlink_to(os.ttyname(slave))
        os.close(slave)

        def emulate():
            buffer = b''
            while not shutdown.is_set():
                try:
                    if not select.select([master], [], [], .05)[0]:
                        continue
                    buffer += os.read(master, 4096)
                except OSError:
                    if not device.exists():
                        return
                    time.sleep(.005)
                    continue
                while len(buffer) >= 20:
                    tx, buffer = buffer[:20], buffer[20:]
                    assert tx[:2] == b'\xfe\xee' and crc(tx[:16]) == int.from_bytes(tx[16:], 'little')
                    motor = tx[2] & 15
                    frames.append((motor, tx[2], struct.unpack_from('<i', tx, 8)[0]))
                    if motor == 0 and state['drop']:
                        if state['drop'] > 0:
                            state['drop'] -= 1
                        continue
                    # 故意一直反馈模式0，证明模式差异不会撤销使能。
                    if motor == 0 and not tx[2] & 128:
                        state['timeout'] = 0
                    rx = bytearray(22)
                    rx[:2] = b'\xfc\xee'
                    rx[2] = motor | (state['timeout'] << 7 if motor == 0 else 0)
                    rx[3:6] = bytes([state['temperature'], 125, 24])
                    struct.pack_into('<iIHH', rx, 10, int(state['rotor'] * 32768 / (2 * math.pi)),
                                     state['error'], int(state['angle'] % 360 * 8192 / 360), 0)
                    try:
                        os.write(master, rx + struct.pack('<I', crc(rx[2:])))
                    except OSError:
                        return
        worker = threading.Thread(target=emulate, daemon=True)
        worker.start()
        return master, worker

    def query(command):
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
            client.settimeout(2)
            client.connect(str(endpoint))
            client.send(command.encode())
            return client.recv(16384).decode()

    def status():
        return dict(item.split('=', 1) for item in query('status').splitlines()[1].split())

    def until(predicate):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('condition timed out: ' + query('status'))

    def ok(command):
        reply = query(command)
        assert reply.startswith('OK'), reply

    master, worker = new_serial()
    log = open(root / 'daemon.log', 'w+')
    process = subprocess.Popen([str(root / 'robotd'), str(device)], cwd=root, stdout=log, stderr=log)
    try:
        until(lambda: endpoint.exists())
        until(lambda: status()['online'] == '1')
        assert status()['enabled'] == '0'
        ok('joint arm 0')
        until(lambda: status()['tx_mode'] == '1')
        for angle in [2, 30, -30, 50]:
            ok(f'joint move 0 {angle}')
        ok('joint arm 0')
        assert status()['target_deg'] == '50'
        for value in ['61', '-41', 'nan', 'inf', 'oops']:
            assert query('joint move 0 ' + value).startswith('IGNORED')
        assert status()['target_deg'] == '50'
        result = subprocess.run(['python3', str(root / 'robotctl'), 'joint', 'move', '0', 'nan'], capture_output=True, text=True)
        assert result.returncode == 0 and result.stdout.startswith('IGNORED')
        ok('joint params 0 2 .03')
        assert status()['kp'] == '2' and status()['target_deg'] == '50'
        state.update(angle=65, error=512, temperature=100)
        until(lambda: float(status()['joint_deg']) > 60)
        assert status()['enabled'] == '1'
        ok('joint move 0 0')
        failures = int(status()['failures'])
        state['drop'] = 1
        until(lambda: int(status()['failures']) > failures)
        until(lambda: status()['online'] == '1')
        assert status()['enabled'] == '1' and status()['target_deg'] == '0'
        state['drop'] = -1
        until(lambda: status()['reference'] == '0')
        assert status()['enabled'] == '1'
        state.update(angle=20, rotor=500, drop=0)
        until(lambda: status()['reference'] == '1')
        assert abs(float(status()['target_deg']) - 20) < .05
        ok('joint move 0 30')
        state['timeout'] = 1
        until(lambda: abs(float(status()['target_deg']) - 20) < .05)
        until(lambda: status()['timeout_flag'] == '0' and next(frame for frame in reversed(frames) if frame[0] == 0)[1] & 128)
        ok('joint move 0 30')
        # 关闭PTY模拟USB拔出，换同一路径的新PTY；不触碰实物。
        device.unlink()
        os.close(master)
        worker.join(timeout=1)
        until(lambda: 'bus_fault=1' in query('status'))
        assert status()['enabled'] == '1' and status()['reference'] == '0'
        state.update(angle=5, rotor=900)
        master, worker = new_serial()
        until(lambda: status()['reference'] == '1')
        assert abs(float(status()['target_deg']) - 5) < .05
        assert process.poll() is None
        ok('joint stop 0')
        until(lambda: status()['tx_mode'] == '0')
        assert status()['enabled'] == '0'
        ok('joint arm 1')  # 未标定也能保持，但不能接收机械目标。
        assert query('joint move 1 0').startswith('ERR')
        ok('lock-all')
        ok('joint arm 0')
        process.send_signal(signal.SIGTERM)
        process.wait(timeout=3)
        process = subprocess.Popen([str(root / 'robotd'), str(device)], cwd=root, stdout=log, stderr=log)
        until(lambda: endpoint.exists())
        until(lambda: status()['online'] == '1')
        assert status()['enabled'] == '0' and status()['kp'] == '2'
        print('PASS: PTY/CRC/IPC/CLI, latest target, single loss, three-loss offline, reset flag, USB reopen, stop, restart disabled, persisted gains')
    finally:
        process.send_signal(signal.SIGTERM)
        process.wait(timeout=3)
        shutdown.set()
        os.close(master)
        worker.join(timeout=1)
        log.seek(0)
        print(log.read())
        log.close()
