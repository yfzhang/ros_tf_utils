import tf.transformations as transformations
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import warnings


def odom_to_mat44(odom):
    """
    Convert odom msg to 4*4 transformation matrix
    :param odom:
    :return:
    """
    # Be careful Odometry msg contains PoseWithCovariance msg
    return pose_to_mat44(odom.pose.pose)


def pose_to_mat44(pose):
    """
    Convert pose msg to 4*4 transformation matrix
    :param pose:
    :return:
    """
    [x, y, z] = [pose.position.x, pose.position.y, pose.position.z]
    trans_mat = transformations.translation_matrix([x, y, z])

    [x, y, z, w] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot_mat = transformations.quaternion_matrix([x, y, z, w])

    return np.dot(trans_mat, rot_mat)


def mat44_to_pose(mat):
    """
    Convert 4*4 transformation matrix to pose msg (no header information added)
    :param mat:
    :return:
    """
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


def euler_to_mat44(roll, pitch, yaw):
    warnings.warn('please use this carefully, because the euler angle axis can be very confusing.')
    return transformations.euler_matrix(roll, pitch, yaw)


def enu2rviz(orig_mat):
    """
    Take a 4*4 matrix that follows ENU definition and output a matrix that follows RVIZ definition
    :param orig_mat:
    :return:
    """
    mat = np.identity(4, dtype=np.float64)
    mat[0][0] = 0.0
    mat[0][1] = 1.0
    mat[1][0] = -1.0
    mat[1][1] = 0.0

    new_mat = mat.dot(orig_mat).dot(np.linalg.inv(mat))
    return new_mat
