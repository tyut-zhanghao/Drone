import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped
import socket
import struct
from time import sleep


MAX_LIN_VEL = 20
MAX_ANGLE = 1
LIN_VEL_STEP_SIZE = 0.1
ANGLE_STEP_SIZE = 0.03

cmd_vel_mask = False
ctrl_leader = False

msg2all = """
a: increase left_trust
d: increase right_trust
q: inrease left_angle
e: increase right_angle
"""

msg2leader = """
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

def print_msg():
    if ctrl_leader:
        print(msg2leader)
    else:
        print(msg2all)

def gazebo_model_state_callback(msg):
    global local_pose
    id = msg.name.index("wamv")
    local_pose.header.stamp = rospy.Time().now()
    local_pose.header.frame_id = 'map'
    local_pose.pose = msg.pose[id]

def client():

    udp_addr = ('<broadcast>', 9999)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

    buf = struct.pack('ff?',local_pose.pose.position.x,local_pose.pose.position.y,True)

    for i in range(10):

        udp_socket.sendto(buf, udp_addr)

        x,y,sign = struct.unpack('ff?',buf)
        print(x,y)
        sleep(0.1)
  
    udp_socket.close()


if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('ugv_keyboard_control')
    local_pose = PoseStamped()


    leader_cmd_vel_pub = rospy.Publisher("/wamv/thrusters/left_thrust_angle", Float32, queue_size=1)
    leader_cmd_vel_pub1 = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size=1)
    leader_cmd_vel_pub2= rospy.Publisher("/wamv/thrusters/right_thrust_angle", Float32, queue_size=1)
    leader_cmd_vel_pub3 = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size=1)

    leader_cmd_vel_pub4 = rospy.Publisher("/wamv/joint_states", Float64, queue_size=1)

    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)



    left_angle=Float32()
    right_angle=Float32()
    left_trust=Float32()
    right_trust=Float32()

    Position=Float64()

    leftAngle=0.0
    rightAngle=0.0
    leftTrust=0.0
    righTrust=0.0

    position=0.0

    print_msg()
    # while(1):
    while not rospy.is_shutdown():

        client()
        key = getKey()
        
        if key == 'a' :
            leftTrust = leftTrust + LIN_VEL_STEP_SIZE
            print_msg()
            print("currently:\t leftTrust %.2f\t " % (leftTrust))
        elif key == 'd' :
            righTrust = righTrust + LIN_VEL_STEP_SIZE
            print_msg()
            print("currently:\t righTrust  %.2f\t " % (righTrust))
        elif key == 'q' :
            leftAngle = leftAngle + ANGLE_STEP_SIZE
            print_msg()
            print("currently:\t leftAngle %.2f\t " % (leftAngle))
        elif key == 'e' :
            rightAngle = rightAngle + ANGLE_STEP_SIZE
            print_msg()
            print("currently:\t rightAngle %.2f\t " % (rightAngle))
        elif key == 'p' :
            position = position + ANGLE_STEP_SIZE
            print_msg()
            print("currently:\t position %.2f\t " % (position))
        elif key == 's' :
            cmd_vel_mask = False
            leftAngle=0.0
            rightAngle=0.0
            leftTrust=0.0
            righTrust=0.0
            
        if leftTrust > MAX_LIN_VEL:
            leftTrust = MAX_LIN_VEL
        elif leftTrust < -MAX_LIN_VEL:
            leftTrust = -MAX_LIN_VEL
        if leftAngle > MAX_ANGLE:
            leftAngle = MAX_ANGLE
        elif leftAngle < -MAX_ANGLE:
            leftAngle = -MAX_ANGLE
            
        if righTrust > MAX_LIN_VEL:
            righTrust = MAX_LIN_VEL
        elif righTrust < -MAX_LIN_VEL:
            righTrust = -MAX_LIN_VEL
        if rightAngle > MAX_ANGLE:
            rightAngle = MAX_ANGLE
        elif rightAngle < -MAX_ANGLE:
            rightAngle = -MAX_ANGLE


        left_angle.data=leftAngle
        right_angle.data=rightAngle
        left_trust.data=leftTrust
        right_trust.data=righTrust
        Position.data=position
        
        
        leader_cmd_vel_pub.publish(left_angle)
        leader_cmd_vel_pub1.publish(left_trust)
        leader_cmd_vel_pub2.publish(right_angle)
        leader_cmd_vel_pub3.publish(right_trust)
        leader_cmd_vel_pub4.publish(position)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
