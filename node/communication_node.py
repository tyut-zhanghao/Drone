# from mavsend import MAVSend
import socket
from udp_mavlink_client import*
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String,Bool

class MAVSend(object):

    udp_addr = ('<broadcast>', 9999)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

    def write(self, buf):
        send_callback = self.udp_socket.sendto(buf, self.udp_addr)
        # print('send')

def planeServer():
    # global broadcastx,broadcasty,sign

    sys_id = 254
    cmp_id = 1

    cur_mav_msg = None
    udp_addr = ('', 9999)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    udp_socket.settimeout(0.1)
    # udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVTIMEO,10)

    udp_socket.bind(udp_addr)

    mav = MAVLink(MAVSend, sys_id, cmp_id)

    data = udp_socket.recvfrom(1024)
    # print(data[0])
    if len(data)>0:
                try:
                    cur_mav_msg = mav.parse_char(data[0])
                except Exception as e:
                    pass
                if isinstance(cur_mav_msg, MAVLink_message):

                    pass

                if isinstance(cur_mav_msg, MAVLink_local_position_ned_message):

                    broadcastx  = cur_mav_msg.x
                    broadcasty = cur_mav_msg.y
                    sign = True
                    # print(broadcastx,broadcasty,"server")
                    # print(sign,"server111")
    udp_socket.close()
    return broadcastx,broadcasty,sign

def wamvServer():
    # global broadcastx,broadcasty
    udp_addr = ('', 9999)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    udp_socket.settimeout(1)

    udp_socket.bind(udp_addr)

    recv_data = udp_socket.recvfrom(1024)

    broadcastx,broadcasty,signWamv = struct.unpack('ff?',recv_data[0])
    udp_socket.close()
    return broadcastx,broadcasty,signWamv


def consoleServer():
    udp_addr = ('', 9998)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    udp_socket.settimeout(0.1)

    udp_socket.bind(udp_addr)

    recv_data = udp_socket.recvfrom(1024)
    signCmd,cmd = struct.unpack('?s',recv_data[0])
    print(cmd)
    print(signCmd)

    udp_socket.close()
    return cmd,signCmd
    #rostopic

if __name__ == "__main__":

    rospy.init_node('communication_node')
    planePosition = PoseStamped()
    wamvPosition = PoseStamped()
    command = String()
    signPlane = bool
    signWamv = bool
    signCmd = bool
    plane_broadcastx = float
    plane_broadcasty = float  
    wamv_broadcastx = float
    wamv_broadcasty = float
    plane_position_pub = rospy.Publisher('/communication_node/planePosition', PoseStamped, queue_size=1)
    plane_other_pub = rospy.Publisher('/plane_other_pose', Bool, queue_size=1)
    wamv_position_pub = rospy.Publisher('/communication_node/wamvPosition', PoseStamped, queue_size=1)
    command_pub = rospy.Publisher('/command',String,queue_size=1)
    while not rospy.is_shutdown():
        try:
            wamv_broadcastx,wamv_broadcasty,signWamv = wamvServer()
        except Exception as e:
            pass

        try:
            plane_broadcastx,plane_broadcasty,signPlane = planeServer()
        except Exception as e:
            pass

        try:
            print("111")
            command,signCmd = consoleServer()
        except Exception as e:
            pass
    
        if(signPlane):
            planePosition.pose.position.x = plane_broadcastx
            planePosition.pose.position.y = plane_broadcasty
            plane_position_pub.publish(planePosition)
            plane_other_pub.publish(True)
            signPlane = False
        if(signWamv):
            wamvPosition.pose.position.x = wamv_broadcastx
            wamvPosition.pose.position.y = wamv_broadcasty
            wamv_position_pub.publish(wamvPosition)
        if(signCmd):
            if(command == "t"):
                command_pub.publish("wamvTrack")
            elif(command == "l"):
                command_pub.publish("land")
            elif(command == "y"):
                command_pub.publish("yoloTrack")
            elif(command == "s"):
                command_pub.publish("stop")