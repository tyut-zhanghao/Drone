from ast import Str

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State
import sys 
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String,Bool

sys.path.append('/home/zhanghao/catkin_ws/devel/lib/python2.7/dist-packages')


import numpy as np
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import MountControl,State

# from mavlink import *
from time import sleep
# from udp_mavlink_client import*

sys.path.append('..')
# from udp_module.client import*
# from udp_module.server import*
from tf2_ros import TransformListener, Buffer
  

def mavros_state_callback(data):
    global mavros_state
    mavros_state = data

def local_pose_callback(data):
    global mavros_local_pose
    mavros_local_pose = data


def gazebo_model_state_callback(msg):
    global local_pose
    id = msg.name.index("typhoon_h480_"+vehicle_id)
    local_pose.header.stamp = rospy.Time().now()
    local_pose.header.frame_id = 'map'
    local_pose.pose = msg.pose[id]
    # print(local_pose.pose.position.x,local_pose.pose.position.y)

def position_track_callback(msg):
    global sign
    sign = msg

def wamv_position_callback(msg):
    global target_pose
    target_pose = msg

if __name__ == "__main__":

    vehicle_id = sys.argv[1]

    broadcastx = float()
    broadcasty = float()
    Kp = 1
    command = ''
    land_vel = 0.5

    target_pose = PoseStamped()

    cmd_vel_enu = Twist()

    local_pose = PoseStamped()

    mavros_local_pose = PoseStamped()
    rospy.init_node('track_wamv_node'+vehicle_id)

    sign = Bool()

    tfBuffer = Buffer()
    tflistener = TransformListener(tfBuffer)

    cmd_vel_pub = rospy.Publisher('/xtdrone/typhoon_h480_'+vehicle_id+'/cmd_vel_enu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/typhoon_h480_'+vehicle_id+'/cmd', String, queue_size=1)
    
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
  
    rospy.Subscriber("typhoon_h480_"+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)
    rospy.Subscriber("/typhoon_h480_"+vehicle_id+"/mavros/state", State, mavros_state_callback)
    rospy.Subscriber("/track/wamv_track", Bool, position_track_callback)
   
    rospy.Subscriber("/communication_node/wamvPosition", PoseStamped, wamv_position_callback)

    setModeServer = rospy.ServiceProxy('/typhoon_h480_'+vehicle_id+'/mavros/set_mode', SetMode)

    mountCnt = rospy.Publisher('/typhoon_h480_'+vehicle_id+'/mavros/mount_control/command', MountControl, queue_size=1)

    msg = MountControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    msg.yaw = 0
    msg.pitch=-90
    mountCnt.publish(msg)  
    rate = rospy.Rate(60) 
      
    while not rospy.is_shutdown():

        if(sign.data):
            print("wamv_tracking!!!")
            x = target_pose.pose.position.x-local_pose.pose.position.x
            y = target_pose.pose.position.y-local_pose.pose.position.y
            z = 10-local_pose.pose.position.z
    
            cmd_vel_enu.linear.x = Kp*x
            cmd_vel_enu.linear.y = y*Kp
            cmd_vel_enu.linear.z = z*Kp
            cmd_vel_pub.publish(cmd_vel_enu)

        # elif(command == "land"):
        #     print("landing")
        #     mountCnt.publish(msg)  
        #     try:
        #         tfstamped = tfBuffer.lookup_transform( 'map','tag_0', rospy.Time(0))
        #     except:
        #         continue
        #     # print(tfstamped)
        #     cmd_vel_enu.linear.x = Kp * (tfstamped.transform.translation.x - mavros_local_pose.pose.position.x)
        #     cmd_vel_enu.linear.y = Kp * (tfstamped.transform.translation.y - mavros_local_pose.pose.position.y)
        #     cmd_vel_enu.linear.z = -land_vel
        #     # print(cmd_vel_enu)

        #     cmd_vel_pub.publish(cmd_vel_enu)
        #     rate.sleep()

        elif(not sign.data):
            cmd_vel_enu.linear.x = 0
            cmd_vel_enu.linear.y = 0
            cmd_vel_enu.linear.z = 0
            cmd_vel_pub.publish(cmd_vel_enu)
            print("HOVER")

        else:
            print("waiting command!!!")

        rate.sleep()


        



