import tty, termios
import sys, select, os
import socket
from mavlink import*
from time import sleep
from mavsend import MAVSend

sys.path.append('..')

class UDPSend(object):


    udp_addr = ('<broadcast>', 9999)
    udp_addr2 = ('<broadcast>', 9998)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

    sys_id = 254
    cmp_id = 1
    send = MAVSend()
    mav = MAVLink(send, sys_id, cmp_id)

    def planeClient(self,positionx,positiony):

        for i in range(10):
            msg = MAVLink_local_position_ned_message(time_boot_ms=0.1,x=positionx,y=positiony,z=0,vx=0,vy=0,vz=0)
            self.mav.send(msg)
            sleep(0.1)
  

    def consoleClient(self,buf):
        for i in range(100):
            
            self.udp_socket.sendto(buf, self.udp_addr2)

            sleep(0.01)

msg2all = """
a: increase left_trust
d: increase right_trust
q: inrease left_angle
e: increase right_angle
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    udpClient = UDPSend()
    # key = ''
    print(msg2all)
    while True:
        # print(msg2all)

        key = getKey()
        if(key == 'q'):
            break
        elif(key == 't'):
            print(key)
            buf = struct.pack('?s',True,key)
            udpClient.consoleClient(buf)
        elif(key == 'l'):
            buf = struct.pack('?s',True,key)
            udpClient.consoleClient(buf)
            # print(key)
        elif(key == 'y'):
            print(key)
            buf = struct.pack('?s',True,key)
            udpClient.consoleClient(buf)
        elif(key == 's'):
            buf = struct.pack('?s',True,key)
            udpClient.consoleClient(buf)
        # elif(key == 'e'):
        #     key = "exit"
        #     buf = struct.pack('',True,key)
        #     udpClient.consoleClient(buf)
        else:
            print("Please Input Correct Command")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    