import argparse
import functools
import json
import lcm
import os
import rospy

from geometry_msgs.msg import PoseStamped
from comm import pose3d_t, model_t

lc = lcm.LCM()
#POSE_OFFSET = pose3d_t()
#POSE_OFFSET.quaternion.x = 0
#POSE_OFFSET.quaternion.y = 0
#POSE_OFFSET.quaternion.z = 0.3826834
#POSE_OFFSET.quaternion.w = 0.9238795

MODEL_ROOT = os.path.join(os.path.dirname(__file__), 'models')

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

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, required=True,
                        help='path to the config json')
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_arguments()
    rospy.init_node('listener', anonymous=False)
    with open(args.config, 'r') as f:
        robots = json.load(f)

    # spawn all robots
    model = model_t()
    for robot in robots:
        model.name = robot['name']
        model.path = os.path.join(MODEL_ROOT, robot['type'], 'model.sdf')
        lc.publish('SPAWN_MODEL', model.encode())
        rospy.Subscriber(os.path.join('/vrpn_client_node', robot['vrpn_name'], 'pose'),
                         PoseStamped, functools.partial(update, name=model.name))

    rospy.spin()

