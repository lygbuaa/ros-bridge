#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, socket, struct
# import numpy as np

class ThreadBase(threading.Thread):
    def __init__(self, callback=None, callback_args=None, *args, **kwargs):
        self.preprocess = kwargs.pop('preprocess')
        super(ThreadBase, self).__init__(target=self.loop_with_callback, *args, **kwargs)
        self.callback = callback
        self.callback_args = callback_args

    def loop_with_callback(self):
        while True:
            if self.preprocess is not None:
                self.preprocess()
            if self.callback is not None:
                self.callback(*self.callback_args)

class SocketServer(object):
    def __init__(self, host="127.0.0.1", port=2368, len=1206):
        self.host = host
        self.port = port
        self.data_len = len
        self.msg = None
        self.counter = 0
        # self.dst_addr = ("192.168.1.102", 10000)
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_server.bind((host, port))
        self.socket_server.setblocking(True)
        print("udp server listen on {}:{}".format(self.host, self.port))

    def recv_msg(self):
        self.msg, addr = self.socket_server.recvfrom(self.data_len)
        if not self.msg:
            print("recv error, loop quit")
            self.th.stop()
        recvtime = time.time()
        if self.counter % 8000 == 0:
            print("recv [{}] msg from client {}, timestamp {}sec".format(self.counter, addr, recvtime))
        self.counter += 1

    def post_process(self, msg):
        print("this is SocketServer base stub!")

    def start_recv_thread(self, run_background=True):
        try: 
            self.th = ThreadBase(name="recv_thread", preprocess=self.recv_msg, callback=self.post_process, callback_args=(self.msg,))
            self.th.setDaemon(run_background)
            self.th.start()
            print("recv thread start")
        except Exception as err:
            print("start_recv_thread error: {}".format(err))

class SocketClient(object):
    def __init__(self, host="127.0.0.1", port=2369):
        self.host = host
        self.port = port
        self.dst_addr = (host, port)
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("udp server will send to {}:{}".format(self.host, self.port))

    def send_list(self, data_list=[]):
        msg = bytearray(data_list)
        print("send msg: {}".format(msg))
        self.socket_client.sendto(msg, self.dst_addr)

    def send_array(self, data_array):
        self.socket_client.sendto(data_array, self.dst_addr)


def run_test1():
    s_server = SocketServer(host="127.0.0.1", port=2368)
    s_server.start_recv_thread()
    s_client = SocketClient(host="127.0.0.1", port=2368)
    s_client.send_byte(data_list=[1, 2, 3])
    time.sleep(1)
    s_client.send_byte(data_list=[4, 5, 6, 7])
    time.sleep(1)
    s_client.send_byte(data_list=[8])
    time.sleep(1)

def run_test2():
    s_server = SocketServer(host="127.0.0.1", port=2368, len=1206)
    s_server.start_recv_thread()
    s_client = SocketClient(host="127.0.0.1", port=2368)
    while True:
        s_client.send_array(bytearray(1206))
        time.sleep(0.1)

def run_test3():
    class SocketServerLidar(SocketServer):
        def __init__(self, host="127.0.0.1", port=2368, len=1206):
            super(SocketServerLidar, self).__init__(host=host, port=port, len=len)

        # the msg passed in is not used
        def post_process(self, msg):
            print("this is SocketServerLidar post process")
            if self.msg is not None:
                header = self.msg[0:7]
                print("msg header: %s", str(header))

    s_server = SocketServerLidar(host="127.0.0.1", port=12371, len=1206)
    s_server.start_recv_thread(run_background=True)
    s_client = SocketClient(host="127.0.0.1", port=12371)
    counter = 0
    while True:
        msg = bytearray(1206)
        msg[0] = int(counter/1000)%256
        msg[1] = int(counter/100)%100
        msg[2] = int(counter/10)%10
        msg[3] = counter%10
        s_client.send_array(msg)
        time.sleep(0.1)
        counter += 1

if __name__ == "__main__":
    print("********** running test ************")
    run_test3()