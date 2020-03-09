import rospy
import lcm
import functools

from geometry_msgs.msg import PoseStamped
from comm import pose3d_t, model_t

lc = lcm.LCM()
POSE_OFFSET = pose3d_t()
POSE_OFFSET.quaternion.x = 0
POSE_OFFSET.quaternion.y = 0
POSE_OFFSET.quaternion.z = 0.3826834
POSE_OFFSET.quaternion.w = 0.9238795

def update(data, name):
    msg = pose3d_t()
    msg.position.x = data.pose.position.x
    msg.position.y = data.pose.position.y
    msg.position.z = data.pose.position.z
    msg.quaternion.x = data.pose.orientation.x
    msg.quaternion.y = data.pose.orientation.y
    msg.quaternion.z = data.pose.orientation.z
    msg.quaternion.w = data.pose.orientation.w

    lc.publish(name, msg.encode())

if __name__ == '__main__':
    model = model_t()

    model.name = 'drone1'
    model.path = '/home/alvin/rtviz/cypy_viz/models/quadrotor/model.sdf'
    lc.publish('SPAWN_MODEL', model.encode())
    lc.publish(model.name + 'CALIB', POSE_OFFSET.encode())

    model.name = 'drone2'
    lc.publish('SPAWN_MODEL', model.encode())
    lc.publish(model.name + 'CALIB', POSE_OFFSET.encode())

    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber('/vrpn_client_node/cyphyhousecopter/pose', PoseStamped,
                     functools.partial(update, name='drone1'))
    rospy.Subscriber('/vrpn_client_node/cyphyhousecopter1/pose', PoseStamped,
                     functools.partial(update, name='drone2'))
    rospy.spin()

