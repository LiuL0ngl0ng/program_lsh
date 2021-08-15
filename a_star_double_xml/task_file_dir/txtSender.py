#!/usr/bin/env python
# -*- coding=utf-8 -*-


"""
file: txtSender.py
socket client
"""
import rospy
import socket
import os
import sys
import struct
import time
import signal

def quit(signum, frame):
    print ''
    print 'stop fusion'
    sys.exit()

def socket_client():
    # try:
    #     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     #sk.connect_ex(address)
    #     s.connect_ex(('192.168.43.232', 6666))
    # except socket.error as msg:
    #     print msg
    # rospy.init_node("txtsender")
    # port = rospy.get_param('~port')
    # ip = rospy.get_param('ip')
    path = os.path.expanduser('~')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while s.connect_ex(("192.168.0.111  ", 6666)) != 0:
        print "wating for vehicle to open server ..."
        time.sleep(1)


    print s.recv(1024)

    while 1:
        filepath = path + '/taskfile/KYXZ2018A.txt'
        if os.path.isfile(filepath):
            # 定义定义文件信息。128s表示文件名为128bytes长，l表示一个int或log文件类型，在此为文件大小
            fileinfo_size = struct.calcsize('128sl')
            # 定义文件头信息，包含文件名和文件大小
            fhead = struct.pack('128sl', os.path.basename(filepath),
                                os.stat(filepath).st_size)
            s.send(fhead)
            print 'client filepath: {0}'.format(filepath)

            fp = open(filepath, 'rb')
            while 1:
                data = fp.read(1024)
                if not data:
                    print '{0} file send over...'.format(filepath)
                    break
                s.send(data)
        s.close()
        break
    sys.exit('sending over')


if __name__ == '__main__':
    socket_client()
