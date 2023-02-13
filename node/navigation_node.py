import rospy
from std_msgs.msg import String,Bool
import subprocess
def command_callback(command):
    global cmd
    cmd = command

if __name__ == "__main__":
    rospy.init_node('navigation_node')

    cmd = String()
    # sign = Bool()
    rospy.Subscriber("/command", String, command_callback,queue_size=1)
    yolotrack_pub = rospy.Publisher('/track/yolo_track', Bool, queue_size=1)
    wamvtrack_pub = rospy.Publisher('/track/wamv_track', Bool, queue_size=1)
    wamvland_pub = rospy.Publisher('/land/wamv_land', Bool, queue_size=1)

    # subprocess.Popen("")
    while not rospy.is_shutdown():
        print(cmd)
        if(cmd.data == "yoloTrack"):
            print("pub")
            yolotrack_pub.publish(True)
        elif(cmd.data == "wamvTrack"):
            print("pub")
            wamvtrack_pub.publish(True)
        elif(cmd.data == "land"):
            wamvtrack_pub.publish(False)
            wamvland_pub.publish(True)
        elif(cmd.data == "stop"):
            yolotrack_pub.publish(False)
            wamvtrack_pub.publish(False)
            wamvtrack_pub.publish(False)
            # wamvland_pub.publish(True)