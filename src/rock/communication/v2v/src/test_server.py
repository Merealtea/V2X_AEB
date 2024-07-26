#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
# AUTHOR: Haoran Wu
# FILE: test_server.py
# DATE: 2022/01/04 周二
# TIME: 21:41:57
'''

import socket, time, threading, sys

host = '' # host of server
port = 8900
SIZE = 2048
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((host, port))
def func(msg, addr):
    print('Connect to {}'.format(addr))
    print('Message form client {}'.format(msg))
    while True:
        t = time.time()
        try:
            s.sendto(str(t).encode('utf-8'),addr)
        except:
            print('connection to {} closed'.format(addr))
            sys.exit(-1)
        time.sleep(0.5)
while True:
    msg, addr = s.recvfrom(SIZE)
    t = threading.Thread(target=func, args=(msg, addr))
    t.start()
