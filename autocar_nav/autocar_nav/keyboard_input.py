#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import termios
import tty
import sys
from select import select
import threading

class KeyboardInput(threading.Thread):
    def __init__(self, rate):
        super(KeyboardInput, self).__init__()
        self.condition = threading.Condition()

        self.rate = rate
        self.settings = termios.tcgetattr(sys.stdin)

        self.start()
        return
    
    def update(self):
        self.condition.acquire()

        key = self.getKey(self.settings, self.rate)

        # # Ctrl + C
        # if (key == '\x03'):
        #     self.join()
        #     return
        
        # elif (key == 'a'):
        #     self.status += 1
            
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()
        return key

    # def getKey(self, settings, timeout):
    #     if sys.platform == 'win32':
    #         # getwch() returns a string on Windows
    #         key = msvcrt.getwch()
    #     else:
    #         tty.setraw(sys.stdin.fileno())
    #         # sys.stdin.read() returns a string on Linux
    #         # rlist, _, _ = select([sys.stdin], [], [], timeout)
    #         # if rlist:
    #         #     key = sys.stdin.read(1)
    #         # else:
    #         #     key = ''
    #         key = sys.stdin.read(1)
    #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    #     return key
    def getKey(self, settings, timeout):
        tty.setraw(sys.stdin.fileno())
        
        # select()를 사용하여 입력이 있을 때만 sys.stdin.read(1) 실행
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else None  # 입력 없으면 None 반환
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    
    @staticmethod
    def keyboard_input_is_none():
        return