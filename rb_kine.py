import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from copy import deepcopy
from math import pi, sin, cos, sqrt

d2r = pi/180
r2d = 180/pi

class ThreeJointLeg(object):
    def __init__(
        self,
        name,
        start_pose,
        link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
        joint_angles={"q1": 0, "q2": 40, "q3": -60},
    ):
        super().__init__()

        self.name = name

        self.setLegJoints(joint_angles)
        self.setLegLength(link_length)
        self.setStartPose(start_pose)

    def setStartPose(self, start_pose):

        self.start_pose = start_pose
        self.calcLegPose()

    def setLegJoints(self, angles):

        self.rad_q1 = angles["q1"] * d2r
        self.rad_q2 = angles["q2"] * d2r
        self.rad_q3 = angles["q3"] * d2r

    def setLegLength(self, lengths):

        self.l1 = lengths["l1"]
        self.l2 = lengths["l2"]
        self.l3 = lengths["l3"]

    def getLegPosition(self):
        # IK시 필요함
        pass

    def setLegPosition(self):
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

    def calcLegPose(self):

        self.x_1_pose = self.start_pose @ self.getT0_1()
        self.x_2_pose = self.x_1_pose @ self.getT1_2()
        self.x_3_pose = self.x_2_pose @ self.getT2_3()

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
        # IK시 필요함
        pass 

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
                    joint_angles={"q1": 0, "q2": 40, "q3": -60}
                )

        self.FL = ThreeJointLeg(
                    name="FL", 
                    start_pose=self.FLPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 40, "q3": -60}
                )

        self.RL = ThreeJointLeg(
                    name="RL", 
                    start_pose=self.RLPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 40, "q3": -60}
                )

        self.RR = ThreeJointLeg(
                    name="RR", 
                    start_pose=self.RRPose,
                    link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
                    joint_angles={"q1": 0, "q2": 40, "q3": -60}
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
            self.FR.setStartPose(self.FRPose)
            self.FL.setStartPose(self.FLPose)
            self.RR.setStartPose(self.RRPose)
            self.RL.setStartPose(self.RLPose)
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


def getBodyLine(bodyPoints, ax):
    body_lines = []

    # Construct the body of 4 lines from the first point of each leg (the four corners of the body)
    for i in range(4):
        # For last leg, connect back to first leg point
        if i == 3:
            ind = -1
        else:
            ind = i

        # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
        # appear oriented better
        x_vals = [bodyPoints[ind][0], bodyPoints[ind + 1][0]]
        y_vals = [bodyPoints[ind][1], bodyPoints[ind + 1][1]]
        z_vals = [bodyPoints[ind][2], bodyPoints[ind + 1][2]]
        body_lines.append(ax.plot(x_vals, y_vals, z_vals, color="k")[0])

    return body_lines

def getLegLines(leg_points, ax):
    leg_lines = []

    # Plot color order for leg links: (hip, upper leg, lower leg)
    plt_colors = ['r','c','b']
    for leg in leg_points:
        for i in range(len(leg) - 1):
            # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
            # appear oriented better
            x_vals = [leg[i][0], leg[i+1][0]]
            y_vals = [leg[i][1], leg[i+1][1]]
            z_vals = [leg[i][2], leg[i+1][2]]
            leg_lines.append(ax.plot(x_vals, y_vals, z_vals,color=plt_colors[i])[0])

    return leg_lines

def update_lines(num, coord_data, lines):

    # print(coord_data[0][0])

    line_to_leg__and_link_dict =   {4:(0,0),
                                    5:(0,1),
                                    6:(0,2),
                                    7:(1,0),
                                    8:(1,1),
                                    9:(1,2),
                                    10:(2,0),
                                    11:(2,1),
                                    12:(2,2),
                                    13:(3,0),
                                    14:(3,1),
                                    15:(3,2)}

    for line, i in zip(lines, range(len(lines))):
        # First four lines are the square body
        if i < 4:
            if i == 3:
                ind = -1
            else:
                ind = i

            # 길이가 바뀌지는 않았나 의심해봄
            # print(sqrt(
            #     pow(coord_data[num][ind][0],2) +
            #     pow(coord_data[num][ind][1],2) +
            #     pow(coord_data[num][ind][2],2)
            # )) 
            # 0.186518765811915 => 문제 없음!!

            # print(coord_data[num][ind])
            # break
             
            x_vals = [coord_data[num][ind][0][0], coord_data[num][ind+1][0][0]]
            y_vals = [coord_data[num][ind][0][1], coord_data[num][ind+1][0][1]]
            z_vals = [coord_data[num][ind][0][2], coord_data[num][ind+1][0][2]]

            line.set_data_3d(x_vals, y_vals, z_vals)

        # Next 12 lines are legs
        else:
            leg_num = line_to_leg__and_link_dict[i][0]
            link_num = line_to_leg__and_link_dict[i][1]
            leg_x_vals = [coord_data[num][leg_num][link_num][0], coord_data[num][leg_num][link_num+1][0]]
            leg_y_vals = [coord_data[num][leg_num][link_num][1], coord_data[num][leg_num][link_num+1][1]]
            leg_z_vals = [coord_data[num][leg_num][link_num][2], coord_data[num][leg_num][link_num+1][2]]
            
            line.set_data_3d(leg_x_vals, leg_y_vals, leg_z_vals)

    return lines

if __name__ == "__main__":
    my_robot = QuadrupedRobot()

    hip_positions = my_robot.getBodyPoints()
    # print(hip_positions)
    FRPose = my_robot.getFRPose()

    # my_fr_Leg = ThreeJointLeg(name="FR", start_pose=FRPose)
    leg_points = my_robot.getLegPoints()
    
    # print("=====================")
    # print(leg_points)
    # {'FRPoints': 
    # [array([0.1805, 0.047 , 0.    ]), 
    # array([0.1805, 0.1308, 0.    ]), 
    # array([ 0.05194248,  0.1308    , -0.15320889]), 
    # array([ 0.12034651,  0.1308    , -0.34114741])]}

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    # X and Y swap
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_xlim3d([-0.4, 0.4])
    ax.set_ylim3d([-0.2, 0.2])
    ax.set_zlim3d([-0.3, 0.2])

    body_lines = getBodyLine(hip_positions, ax)
    leg_lines = getLegLines(leg_points, ax)
    whole_lines = body_lines + leg_lines
    # coord_data = getPsiCoordData(25, -30, 30, )

    num_angles = 25
    pitch_angles = np.linspace(-30*d2r,30*d2r,num_angles)
    coord_data = []
    for theta in pitch_angles:
        # Set a pitch angle
        my_robot.setCOMOrientation(theta=theta)

        # Get leg coordinates and append to data list
        # coord_data.append(my_robot.getBodyPoints())
        coord_data.append(my_robot.getLegPoints())

        my_robot.resetCOMOrientation()

    coord_data = coord_data + coord_data[::-1]

    line_ani = animation.FuncAnimation(
        fig,
        update_lines,
        num_angles * 2,
        fargs=(coord_data, whole_lines),
        interval=75,
        blit=False,
    )

    plt.show()
