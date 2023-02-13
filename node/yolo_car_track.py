import sys 
sys.path.append('/home/zhanghao/catkin_ws/devel/lib/python2.7/dist-packages')
from ast import Str
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String,Bool
from mavros_msgs.msg import State

from gazebo_msgs.msg import ModelStates

sys.path.append('..')

from pyquaternion import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes
import math

import numpy as np
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import MountControl,State

from time import sleep
from client import UDPSend
def yuntaicall_back(msg):
    global yaw,pitch,roll
    yaw=-(3.1415927/180)*float(msg.yaw)


def darknet_callback(data):
    global find_cnt, twist, cmd, target_height_mask, target_height,theta, get_time,last_u,last_v,flag,udpClient
    for target in data.bounding_boxes:
        if(target.id==1):
            
            theta=1.04719

            flag = True
            udpClient.planeClient(local_pose.pose.position.x,local_pose.pose.position.y)

            setModeServer(custom_mode="OFFBOARD")
            z = height / math.sin(theta)  
            u = (target.xmax+target.xmin)/2
            v = (target.ymax+target.ymin)/2
            u_ = u-u_center
            v_ = v-v_center
            last_u=u_
            last_v=v_
            u_velocity = -Kp_xy*u_
            v_velocity = -Kp_xy*v_
            x_velocity = float(v_velocity*z/(v_*math.cos(theta)+fy*math.sin(theta)))
            y_velocity = float((z*u_velocity-u_*math.cos(theta)*x_velocity)/fx)
            p=np.array([[x_velocity,y_velocity,float(Kp_z*(target_height-height))]])

            yaw_juzhen=np.array([[float(math.cos(yaw)),float(-math.sin(yaw)),0],[float(math.sin(yaw)),float(math.cos(yaw)),0],[0,0,1]])

            p_1=np.dot(yaw_juzhen,p.T)

            twist.linear.x = float(p_1[0][0])
 
            twist.linear.y =float(p_1[1][0])
            cmd = ''
            find_cnt = find_cnt + 1
            get_time = False

def local_pose_callback(data):
    global height, target_height, target_set,feiji_local_pose_yaw
    
    q = Quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
    feiji_local_pose_yaw = q.yaw_pitch_roll[0]
    height = data.pose.position.z    
    if not target_set:
        target_height = height     
        target_set = True    


def mavros_state_callback(data):
    global mavros_state
    mavros_state = data

def gazebo_model_state_callback(msg):
    global local_pose
    id = msg.name.index("typhoon_h480_"+vehicle_id)
    local_pose.header.stamp = rospy.Time().now()
    local_pose.header.frame_id = 'map'
    local_pose.pose = msg.pose[id]

def yolo_track_callback(msg):
    global sign
    sign = msg

def position_callback(msg):
    global target_pose
    target_pose = msg

def plane_other_callback(msg):
    global clue
    clue = msg

if __name__ == "__main__":

    vehicle_id = sys.argv[1]

    udpClient = UDPSend()
    command = ''

    broadcastx = float()
    broadcasty = float()
    target_pose = PoseStamped()

    flag = False
    sign = Bool()
    print(sign)
    clue = Bool()

    height = 0  
    target_height = 0
    target_set = False
    find_cnt = 0
    find_cnt_last = 0
    not_find_time = 0
    get_time = False
    twist = Twist()
    cmd = String()
    theta = 1.04719
    u_center=640/2 
    v_center=360/2
    fx = 205.46963709898583
    fy = 205.46963709898583
    Kp_xy = 0.2
    Kp_z = 1

    pitch=0
    roll=0
    yaw=0
    yaw1=0
    last_u=0
    last_v=0
    i=0
    p_1=np.zeros(3)
    feiji_local_pose_yaw= 0
    mavros_state=State()



    cmd_vel_enu = Twist()

    local_pose = PoseStamped()

    rospy.init_node('yolo_tracking_node'+vehicle_id)
    
    cmd_vel_pub = rospy.Publisher('/xtdrone/typhoon_h480_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/typhoon_h480_'+vehicle_id+'/cmd', String, queue_size=1)
    
    
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
    
    rospy.Subscriber("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback,queue_size=1)
    
    rospy.Subscriber("/typhoon_h480_"+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)

    rospy.Subscriber("/typhoon_h480_"+vehicle_id+"/mavros/state", State, mavros_state_callback)
    rospy.Subscriber("/track/yolo_track", Bool, yolo_track_callback)
    rospy.Subscriber("/communication_node/planePosition", PoseStamped, position_callback)
    rospy.Subscriber("/plane_other_pose", Bool, plane_other_callback)


    mountCnt = rospy.Subscriber('typhoon_h480_'+vehicle_id+'/mavros/mount_control/command', MountControl, yuntaicall_back)
    setModeServer = rospy.ServiceProxy('/typhoon_h480_'+vehicle_id+'/mavros/set_mode', SetMode)
    # setModeServer(custom_mode="AUTO.MISSION")
    mountCnt = rospy.Publisher('/typhoon_h480_'+vehicle_id+'/mavros/mount_control/command', MountControl, queue_size=1)
    msg = MountControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    msg.yaw = yaw1
    msg.pitch=-60
    mountCnt.publish(msg)  
    rate = rospy.Rate(60) 


    while not rospy.is_shutdown():
        print(sign)
        if(sign.data):
            print("track")
            if(flag):
                cmd_vel_pub.publish(twist)
                cmd_pub.publish(cmd)
                # print("0-yolo")
            elif(clue.data):
                x = target_pose.pose.position.x-local_pose.pose.position.x
                y = target_pose.pose.position.y-local_pose.pose.position.y
                z = 5-local_pose.pose.position.z
                cmd_vel_enu.linear.x = x
                cmd_vel_enu.linear.y = y
                cmd_vel_enu.linear.z = z
                cmd_vel_pub.publish(cmd_vel_enu)

            if find_cnt - find_cnt_last == 0:
                if not get_time:
                    not_find_time = rospy.get_time()
                    get_time = True
                if rospy.get_time() - not_find_time > 2.0:
                    i=(i+30)%360
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    cmd = 'HOVER'
                    print(cmd)
                    if last_u>0:
                        print("right")
                        yaw1=i
                    if last_u<0:
                        print("left")
                        yaw1=-i
                    msg = MountControl()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "map"
                    msg.mode = 2
                    msg.yaw = yaw1
                    msg.pitch=-60
                    mountCnt.publish(msg)

                    flag = False
                    cmd_vel_pub.publish(twist)
                    get_time = False
            find_cnt_last = find_cnt    
            rate.sleep()
        elif(not sign.data):
            cmd_vel_enu.linear.x = 0
            cmd_vel_enu.linear.y = 0
            cmd_vel_enu.linear.z = 0
            cmd_vel_pub.publish(cmd_vel_enu)
            print("stop")