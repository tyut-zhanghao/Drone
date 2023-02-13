import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformListener, Buffer
import sys
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import MountControl,State
from std_msgs.msg import String,Bool

sys.path.append('..')


def yuntaicall_back(msg):
    global yaw,pitch,roll
    yaw=-(3.1415927/180)*float(msg.yaw)


def local_pose_callback(data):
    global local_pose
    local_pose = data

def apriltag_land_callback(data):
    global sign
    sign = data

if __name__ == '__main__':

    vehicle_id = sys.argv[1]

    rospy.init_node('precision_landing_node'+vehicle_id)

    tfBuffer = Buffer()
    tflistener = TransformListener(tfBuffer)
    cmd_vel_enu = Twist()   
    local_pose = PoseStamped()
    Kp = 1.0
    land_vel = 0.5
    sign = Bool()

    rospy.Subscriber("typhoon_h480_"+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)

    cmd_vel_pub = rospy.Publisher('/xtdrone/typhoon_h480_'+vehicle_id+'/cmd_vel_enu', Twist, queue_size=1)

    mountCnt = rospy.Subscriber('typhoon_h480_'+vehicle_id+'/mavros/mount_control/command', MountControl, yuntaicall_back)
    # setModeServer = rospy.ServiceProxy('/typhoon_h480_0/mavros/set_mode', SetMode)
    # setModeServer(custom_mode="AUTO.MISSION")
    rospy.Subscriber('/land/wamv_land', Bool, apriltag_land_callback)

    mountCnt = rospy.Publisher('/typhoon_h480_'+vehicle_id+'/mavros/mount_control/command', MountControl, queue_size=1)
    rate = rospy.Rate(60) 
    msg = MountControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    msg.yaw = 0
    msg.pitch=-90
    mountCnt.publish(msg)  

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        mountCnt.publish(msg)  
        try:
            tfstamped = tfBuffer.lookup_transform( 'map','tag_'+vehicle_id, rospy.Time(0))
        except:
            continue
        if(sign.data):
            cmd_vel_enu.linear.x = Kp * (tfstamped.transform.translation.x - local_pose.pose.position.x)
            cmd_vel_enu.linear.y = Kp * (tfstamped.transform.translation.y - local_pose.pose.position.y)
            cmd_vel_enu.linear.z = -land_vel
            # print(cmd_vel_enu)

            cmd_vel_pub.publish(cmd_vel_enu)
            rate.sleep()
        # elif(command == "exit"):
        #     break
        # else:
        #     print("waiting command!!!")
