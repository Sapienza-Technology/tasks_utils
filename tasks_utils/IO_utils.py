import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

import os

import numpy as np

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def get_style_string(msg,style):
    return style+msg+bcolors.ENDC

def style_print(msg,style):
    print(style+msg+bcolors.ENDC)

def logERR(msg):
    style_print(msg,bcolors.FAIL)

def logOK(msg):
    style_print(msg,bcolors.OKGREEN)

def logINFO(msg):
    style_print(msg,bcolors.OKBLUE)

def logWARN(msg):
    style_print(msg,bcolors.WARNING)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if key=='\x03':
            raise KeyboardInterrupt
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')



#********** MESSAGES **********
boot_msg = """
********** CAROTATORE **********
Press:
W and S arrow:  move drill up and down
> and < :  increase and decrease drilling speed
+ and - to calibrate deep scale sample
* and _ to calibrate surface scale sample
G to change semaphore color to Green
R to change semaphore color to Red
B to change semaphore color to Blue
Y to change semaphore color to Yellow
P to change semaphore color to Purple
SPACE to stop
"""


stats_msg = """
Status:
LED status: {0} \tsurface sample: {1} g\tdeep sample: {2} g
"""
