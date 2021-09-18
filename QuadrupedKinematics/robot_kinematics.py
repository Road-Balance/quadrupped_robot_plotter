import numpy as np

from .leg_kinematics import ThreeJointLeg
from math import pi, sin, cos

d2r = pi/180
r2d = 180/pi

class QuadrupedRobot(object):
    def __init__(
        self,
        position={"x": 0, "y": 0, "z": 0},
        link_length={"hip_len": 0.0838, "upper_len": 0.2, "lower_len": 0.2},
        body_length={"width": 0.094, "height": 0.361},
        orientation={"phi": 0, "theta": 0, "psi": 0},
    ):
        super().__init__()

        self.is_first = True

        self.originPoseMat = np.eye(4)

        self.setCOMPosition(position)
        self.setRobotParams(
            link_length["hip_len"],
            link_length["upper_len"],
            link_length["lower_len"],
            body_length["width"],
            body_length["height"],
        )

        self.setCOMOrientation(
            orientation["phi"], orientation["theta"], orientation["psi"]
        )

        self.FR = ThreeJointLeg(
                    name="FR", 
                    start_pose=self.FRPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 30 * d2r, "q3": -60 * d2r}
                )

        self.FL = ThreeJointLeg(
                    name="FL", 
                    start_pose=self.FLPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 30 * d2r, "q3": -60 * d2r}
                )

        self.RL = ThreeJointLeg(
                    name="RL", 
                    start_pose=self.RLPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 30 * d2r, "q3": -60 * d2r}
                )

        self.RR = ThreeJointLeg(
                    name="RR", 
                    start_pose=self.RRPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 30 * d2r, "q3": -60 * d2r}
                )

        self.leg_points = {"FR" : [], "FL" : [], "RR" : [], "RL" : []}

    def setCOMPosition(self, position):

        self.x = position["x"]
        self.y = position["y"]
        self.z = position["z"]

        # TODO: update origin poseMat

    def setRobotParams(
        self, hip_len=0.0838, upper_len=0.2, lower_len=0.2, width=0.094, height=0.361
    ):

        self.hip_length = hip_len
        self.upper_leg_length = upper_len
        self.lower_leg_length = lower_len
        self.body_width = width
        self.body_length = height

        # TODO: Update Hip Points

    def setLegPoints(self, leg_points):

        if self.is_first == True:
            raise Exception('First Setting Error') 

        self.FR.setLegPoint(leg_points["FR"])
        self.FL.setLegPoint(leg_points["FL"])
        self.RR.setLegPoint(leg_points["RR"])
        self.RL.setLegPoint(leg_points["RL"])


    def setCOMOrientation(self, phi=0, theta=0, psi=0):

        # x =>  phi   => Roll
        # y =>  theta => Pitch
        # z =>  psi   => Yaw

        self.Roll = np.array(
            [
                [1, 0, 0, 0],
                [0, cos(phi), -sin(phi), 0],
                [0, sin(phi), cos(phi), 0],
                [0, 0, 0, 1],
            ]
        )

        self.Pitch = np.array(
            [
                [cos(theta), 0, sin(theta), 0],
                [0, 1, 0, 0],
                [-sin(theta), 0, cos(theta), 0],
                [0, 0, 0, 1],
            ]
        )

        self.Yaw = np.array(
            [
                [cos(psi), -sin(psi), 0, 0],
                [sin(psi), cos(psi), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        self.originPoseMat = self.Yaw @ self.Pitch @ self.Roll @ self.originPoseMat
        self.updateWholePoints()

    def resetCOMOrientation(self):

        self.originPoseMat = np.eye(4)
        self.updateWholePoints()

    def updateWholePoints(self):

        if self.is_first == True:
            self.calcHipPose()
            self.is_first = False
        else:
            # 나중에는 IK로 바뀌어야 한다.
            self.calcHipPose()
            # self.FR.setStartPose(self.FRPose)
            # self.FL.setStartPose(self.FLPose)
            # self.RR.setStartPose(self.RRPose)
            # self.RL.setStartPose(self.RLPose)
            self.FR.resetStartPose(self.FRPose)
            self.FL.resetStartPose(self.FLPose)
            self.RR.resetStartPose(self.RRPose)
            self.RL.resetStartPose(self.RLPose)
            pass
        
    def getFRPose(self):

        T_FR = np.array(
            [
                [1, 0, 0, +self.body_length / 2],
                [0, 1, 0, -self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_FR

    def getFLPose(self):

        T_FL = np.array(
            [
                [1, 0, 0, +self.body_length / 2],
                [0, 1, 0, +self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_FL

    def getRRPose(self):

        T_RR = np.array(
            [
                [1, 0, 0, -self.body_length / 2],
                [0, 1, 0, -self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_RR

    def getRLPose(self):

        T_RL = np.array(
            [
                [1, 0, 0, -self.body_length / 2],
                [0, 1, 0, +self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_RL

    def calcHipPose(self):

        self.FRPose = self.originPoseMat @ self.getFRPose()
        self.FLPose = self.originPoseMat @ self.getFLPose()
        self.RRPose = self.originPoseMat @ self.getRRPose()
        self.RLPose = self.originPoseMat @ self.getRLPose()

    def getHipPose(self):

        self.legPose = {
            "FRPose": self.FRPose,
            "FLPose": self.FLPose,
            "RRPose": self.RRPose,
            "RLPose": self.RLPose,
        }

        return self.legPose

    def getBodyPoints(self):

        FRPosition = self.FRPose[0:3, 3]
        FLPosition = self.FLPose[0:3, 3]
        RRPosition = self.RRPose[0:3, 3]
        RLPosition = self.RLPose[0:3, 3]

        # 실제로는 Z자 모양 순서인데, 그림을 위해서 조작한다.
        return [
            FRPosition,
            FLPosition,
            RLPosition,
            RRPosition,
        ]
    
    def getLegPoints(self):

        self.legPoints = (
            self.FR.getLegPoints(),
            self.FL.getLegPoints(),
            self.RL.getLegPoints(),
            self.RR.getLegPoints(),
        )
    
        return self.legPoints

if __name__ == "__main__":
    my_robot = QuadrupedRobot()
