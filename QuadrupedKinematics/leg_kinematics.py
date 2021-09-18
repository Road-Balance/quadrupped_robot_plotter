import numpy as np
from math import pi, sin, cos, sqrt, atan2, acos

d2r = pi/180
r2d = 180/pi

def ht_inverse(ht):
    '''Calculate the inverse of a homogeneous transformation matrix

    The inverse of a homogeneous transformation matrix can be represented as a
    a matrix product of the following:

                -------------------   ------------------- 
                |           |  0  |   | 1   0   0  -x_t |
    ht_inv   =  |   R^-1    |  0  | * | 0   1   0  -y_t |
                |___________|  0  |   | 0   0   1  -z_t |
                | 0   0   0 |  1  |   | 0   0   0   1   |
                -------------------   -------------------

    Where R^-1 is the ivnerse of the rotation matrix portion of the homogeneous
    transform (the first three rows and columns). Note that the inverse
    of a rotation matrix is equal to its transpose. And x_t, y_t, z_t are the
    linear trasnformation portions of the original transform.    
    
    Args
        ht: Input 4x4 nump matrix homogeneous transformation

    Returns:
        A 4x4 numpy matrix that is the inverse of the inputted transformation
    '''
    # Get the rotation matrix part of the homogeneous transform and take the transpose to get the inverse
    temp_rot = ht[0:3,0:3].transpose()

    # Get the linear transformation portion of the transform, and multiply elements by -1
    temp_vec = -1*ht[0:3,3]

    # Block the inverted rotation matrix back to a 4x4 homogeneous transform matrix
    temp_rot_ht = np.block([ [temp_rot            ,   np.zeros((3,1))],
                             [np.zeros((1,3))     ,         np.eye(1)] ])

    # Create a linear translation homogeneous transformation matrix 
    temp_vec_ht = np.eye(4)
    temp_vec_ht[0:3,3] = temp_vec

    # Return the matrix product
    return temp_rot_ht @ temp_vec_ht

class ThreeJointLeg(object):
    def __init__(
        self,
        name,
        start_pose,
        link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
        joint_angles={"q1": 0, "q2": 30 * d2r, "q3": -60 * d2r},
    ):
        super().__init__()

        self.name = name
        self.is_first = True

        self.setLegJointAngles(joint_angles)
        self.setLegLength(link_length)
        self.setStartPose(start_pose)

    def setStartPose(self, start_pose):

        self.start_pose = start_pose
        self.calcFK()

    def resetStartPose(self, start_pose):

        self.start_pose = start_pose
        # inverse of start_pose
        leg_inv = ht_inverse(self.start_pose)
        # inverse @ end_point => new ik point
        new_ik_point = leg_inv @ self.end_leg_point
        
        # [-0.06015349 -0.0838     -0.34114741  1.        ]
        
        # # ik with new ik point
        self.rad_q1, self.rad_q2, self.rad_q3 = self.calcIK(new_ik_point)
        joint_angles={
            "q1": self.rad_q1, 
            "q2": self.rad_q2,
            "q3": self.rad_q3
        }

        self.setLegJointRadians(joint_angles)
        self.calcFK()

    def setLegPoint(self, point):
        # IK시 필요함
        self.rad_q1, self.rad_q2, self.rad_q3 = self.calcIK(point)
        joint_angles={
            "q1": self.rad_q1, 
            "q2": self.rad_q2,
            "q3": self.rad_q3
        }

        self.setLegJointRadians(joint_angles)
        self.calcFK()

    def setLegJointRadians(self, radians):

        self.rad_q1 = radians["q1"]
        self.rad_q2 = radians["q2"]
        self.rad_q3 = radians["q3"]

    def setLegJointAngles(self, angles):

        self.rad_q1 = angles["q1"]
        self.rad_q2 = angles["q2"]
        self.rad_q3 = angles["q3"]

    def setLegLength(self, lengths):

        self.l1 = lengths["l1"]
        self.l2 = lengths["l2"]
        self.l3 = lengths["l3"]

    def getLegPose(self):
        # IK시 필요함
        pass

    def getT0_1(self):

        sign_offset = 1

        if self.name == "FL" or self.name == "RL":
            sign_offset = -1

        _cos = cos(self.rad_q1)
        _sin = sin(self.rad_q1)

        T0_1 = np.array(
            [
                [1, 0, 0, 0],
                [0, _cos, -_sin, sign_offset * -self.l1 * _cos],
                [0, _sin,  _cos, sign_offset * -self.l1 * _sin],
                [0, 0, 0, 1],
            ]
        )

        return T0_1

    def getT1_2(self):

        _cos = cos(self.rad_q2)
        _sin = sin(self.rad_q2)

        T1_2 = np.array(
            [
                [_cos, 0, _sin, -self.l2 * _sin],
                [0, 1, 0, 0],
                [-_sin, 0, _cos, -self.l2 * _cos],
                [0, 0, 0, 1],
            ]
        )

        return T1_2

    def getT2_3(self):

        _cos = cos(self.rad_q3)
        _sin = sin(self.rad_q3)

        T2_3 = np.array(
            [
                [_cos, 0, _sin, -self.l3 * _sin],
                [0, 1, 0, 0],
                [-_sin, 0, _cos, -self.l3 * _cos],
                [0, 0, 0, 1],
            ]
        )

        return T2_3

    def calcFK(self):

        self.x_1_pose = self.start_pose @ self.getT0_1()
        self.x_2_pose = self.x_1_pose @ self.getT1_2()
        self.x_3_pose = self.x_2_pose @ self.getT2_3()

        self.end_leg_point = self.x_3_pose[0:4, 3]

        if self.is_first:
            self.is_first = False
        
    def calcIK(self, end_point):

        if len(end_point) == 3:
            x, y, z = end_point
        else: 
            x, y, z, _ = end_point
        
        r1 = sqrt(y**2 + z**2 - self.l1**2)
        r2 = sqrt(r1**2 + x**2)
        r3 = (r2**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        
        sign_offset = 1

        if self.name == "FL" or self.name == "RL":
            sign_offset = -1


        rad_q1 = atan2(z, y) + atan2(r1, -1 * sign_offset * self.l1)
        
        if r3 > 1:
            r3 = 1
        elif r3 < -1:
            r3 = -1

        rad_q3 = -1 * acos(r3)
        # rad_q3 = -1 * atan2(sqrt(1 - r3 ** 2), r3)
        rad_q2 = atan2(-x, r1) - atan2(self.l3*sin(rad_q3), self.l2 + self.l3 *cos(rad_q3))

        return (
            rad_q1,
            rad_q2,
            rad_q3,
        )

    def getLegPoints(self):

        x_0 = self.start_pose[0:3, 3]
        x_1 = self.x_1_pose[0:3, 3]
        x_2 = self.x_2_pose[0:3, 3]
        x_3 = self.x_3_pose[0:3, 3]

        return [
            x_0,
            x_1,
            x_2,
            x_3,
        ]

    def getLegAngles(self):

        return self.rad_q1, self.rad_q2, self.rad_q3