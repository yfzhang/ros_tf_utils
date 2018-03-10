import tf.transformations as transformations
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry


def odom_to_mat44(odom):
    return pose_to_mat44(odom.pose)


def pose_to_mat44(pose):
    [x, y, z] = [pose.position.x, pose.position.y, pose.position.z]
    trans_mat = transformations.translation_matrix([x, y, z])

    [x, y, z, w] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot_mat = transformations.quaternion_matrix([x, y, z, w])

    return np.dot(trans_mat, rot_mat)


def mat44_to_pose(mat):
    trans = Point(*tuple(transformations.translation_from_matrix(mat)))
    rot = Quaternion(*tuple(transformations.quaternion_from_matrix(mat)))
    return Pose(trans, rot)


def quaternion_to_euler(x, y, z, w):
    q = (x, y, z, w)
    euler = transformations.euler_from_quaternion(q)
    return euler[0], euler[1], euler[2]


def euler_to_quaternion(roll, pitch, yaw):
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    return q[0], q[1], q[2], q[3]
