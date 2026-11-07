#!/usr/bin/env python3
"""SSH标准输入/输出转发到robotd；不打开串口，退出时保持最后目标。"""
import json
import os
import socket
import sys

JOINTS = {"right_ankle": 0, "right_knee": 1}


def query(command):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(2)
        client.connect('/tmp/microduck-%d/robot.sock' % os.getuid())
        client.sendall(command.encode())
        return client.recv(16384).decode()


def status():
    lines = query('status').splitlines()
    if not lines or lines[0].startswith('ERR'):
        raise RuntimeError('\n'.join(lines))
    rows = [dict(item.split('=', 1) for item in line.split()) for line in lines[1:]]
    result = {name: next(row for row in rows if int(row['id']) == id)
              for name, id in JOINTS.items()}
    if any(row['name'] != name for name, row in result.items()):
        raise RuntimeError('ID 0/1的关节名称不匹配，请检查robotd配置')
    return result


def handle(message):
    op = message['op']
    replies = {}
    if op == 'arm':
        rows = status()
        if any(r['calibrated'] != '1' or r['online'] != '1'
               or not 0 <= int(r['age_ms']) <= 200 for r in rows.values()):
            raise RuntimeError('使能需要两个关节均已标定且有新鲜在线反馈')
    if op in ('arm', 'stop'):
        for name in JOINTS:
            replies[name] = query('joint %s %s' % (op, name))
    elif op == 'move':
        for name in JOINTS:
            # Windows传来的是机械角度，板端继续负责已有的限位和编码检查。
            replies[name] = query('joint move %s %.12g' % (name, float(message['degrees'][name])))
    elif op != 'status':
        raise ValueError('unknown operation')
    return {'joints': status(), 'replies': replies}


def main():
    for line in sys.stdin:
        try:
            reply = handle(json.loads(line))
        except (ValueError, KeyError, RuntimeError, OSError, StopIteration) as error:
            reply = {'error': str(error)}
        print(json.dumps(reply, ensure_ascii=True, allow_nan=False), flush=True)
    # EOF/SSH断开不发送stop，robotd保持最后目标。


if __name__ == '__main__':
    main()
