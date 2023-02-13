from udp_mavlink_client import*
from mavsend import MAVSend
import socket
from time import sleep
from geometry_msgs.msg import Twist, PoseStamped


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
        # udp_socket.sendto(buf, udp_addr)
            msg = MAVLink_local_position_ned_message(time_boot_ms=0.1,x=positionx,y=positiony,z=0,vx=0,vy=0,vz=0)
        # buf = msg.pack(mav)
            self.mav.send(msg)
        # udp_socket.sendto(buf, udp_addr)

            sleep(0.1)
  
        # self.udp_socket.close()


    def wamvClient(self,positionx,positiony):

        buf = struct.pack('ff',positionx,positiony)

        for i in range(10):
            
            self.udp_socket.sendto(buf, self.udp_addr)

            x,y = struct.unpack('ff',buf)
            print(x,y)
            sleep(0.1)
  
        # self.udp_socket.close()


    def consoleClient(self,buf):
        # print(buf)
        # buffer = struct.pack('',buf)
        # print(struct.unpack('',buffer))
        for i in range(100):
            
            self.udp_socket.sendto(buf, self.udp_addr2)

            sleep(0.01)
  
        # self.udp_socket.close()
