import rospy
import lcm

from geometry_msgs.msg import PoseStamped
from comm import pose3d_t

msg = pose3d_t()
lc = lcm.LCM()

def update(data):
    msg = pose3d_t()
    msg.position.x = data.pose.position.x
    msg.position.y = data.pose.position.y
    msg.position.z = data.pose.position.z
    msg.quaternion.x = data.pose.orientation.x
    msg.quaternion.y = data.pose.orientation.y
    msg.quaternion.z = data.pose.orientation.z
    msg.quaternion.w = data.pose.orientation.w

    lc.publish('f1tenth_clone_0', msg.encode())

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber('/vrpn_client_node/Test1/pose', PoseStamped, update)
    rospy.spin()

