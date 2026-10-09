"""Does an isolated macOS PTY accept IOSSIOSPEED (pyserial's darwin custom-baud path)?"""
import array, fcntl, os, pty, termios
IOSSIOSPEED = 0x80045402
m, s = pty.openpty()
for sp in (38400, 230400, 1000000, 6000000):
    try:
        fcntl.ioctl(s, IOSSIOSPEED, array.array('i', [sp]), 1)
        print(sp, 'accepted; tcgetattr speed', termios.tcgetattr(s)[4])
    except OSError as e:
        print(sp, 'rejected:', e)
os.close(m); os.close(s)
