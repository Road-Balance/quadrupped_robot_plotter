import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

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
        self.start_pose = start_pose
        start_position = start_pose[0:3, 3]

        self.setHipPosition(start_position)
        self.setLegLength(link_length)
        self.setLegJoints(joint_angles)

    def setHipPosition(self, position):

        self.x = position[0]
        self.y = position[1]
        self.z = position[2]

    def setLegJoints(self, angles):

        self.rad_q1 = angles["q1"] * d2r
        self.rad_q2 = angles["q2"] * d2r
        self.rad_q3 = angles["q3"] * d2r

    def setLegLength(self, lengths):

        self.l1 = lengths["l1"]
        self.l2 = lengths["l2"]
        self.l3 = lengths["l3"]

        print(self.l1, self.l2, self.l3)

    def getLegPosition(self):
        # IK시 필요함
        pass

    def setLegPosition(self):
        # IK시 필요함
        pass


    def getT0_1(self):

        if 'F' in self.name:
            print("Yes")
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

    def getLegPoints(self):

        # self.legPointPose = self.getT2_() @ self.getT1_2() @ self.getT0_1() @ self.start_pose
        x_0 = self.start_pose[0:3, 3]
        
        # x_1_pose = self.getT0_1() @ self.start_pose
        # x_2_pose = self.getT1_2() @ x_1_pose
        # x_3_pose = self.getT2_3() @ x_2_pose

        x_1_pose = self.start_pose @ self.getT0_1()
        x_2_pose = x_1_pose @ self.getT1_2()
        x_3_pose = x_2_pose @ self.getT2_3()

        x_1 = x_1_pose[0:3, 3]
        x_2 = x_2_pose[0:3, 3]
        x_3 = x_3_pose[0:3, 3]


        # 실제로는 Z자 모양 순서인데, 그림을 위해서 조작한다.
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

    def setCOMPosition(self, position):

        self.x = position["x"]
        self.y = position["y"]
        self.z = position["z"]

    def setRobotParams(
        self, hip_len=0.0838, upper_len=0.2, lower_len=0.2, width=0.094, height=0.361
    ):

        self.hip_length = hip_len
        self.upper_leg_length = upper_len
        self.lower_leg_length = lower_len
        self.body_width = width
        self.body_length = height

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

    def resetCOMOrientation(self):

        self.originPoseMat = np.eye(4)

    def getFRPose(self):

        T_FR = np.array(
            [
                [1, 0, 0, +self.body_length / 2],
                [0, 1, 0, +self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_FR

    def getFLPose(self):

        T_FR = np.array(
            [
                [1, 0, 0, +self.body_length / 2],
                [0, 1, 0, -self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_FR

    def getRRPose(self):

        T_FR = np.array(
            [
                [1, 0, 0, -self.body_length / 2],
                [0, 1, 0, +self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_FR

    def getRLPose(self):

        T_FR = np.array(
            [
                [1, 0, 0, -self.body_length / 2],
                [0, 1, 0, -self.body_width / 2],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        return T_FR

    def getHipPose(self):

        self.FRPose = self.originPoseMat @ self.getFRPose()
        self.FLPose = self.originPoseMat @ self.getFLPose()
        self.RRPose = self.originPoseMat @ self.getRRPose()
        self.RLPose = self.originPoseMat @ self.getRLPose()

        self.legPose = {
            "FRPose": self.FRPose,
            "FLPose": self.FLPose,
            "RRPose": self.RRPose,
            "RLPose": self.RLPose,
        }

    def getHipPosition(self):

        self.getHipPose()

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


def getBodyLine(bodyPoints, ax):
    lines = []

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
        lines.append(ax.plot(x_vals, y_vals, z_vals, color="k")[0])

    return lines

def getFRLegLine(fr_leg_points, ax):
    # Plot color order for leg links: (hip, upper leg, lower leg)
    plt_colors = ['r','c','b']
    for i in range(len(fr_leg_points) - 1):
        # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
        # appear oriented better
        x_vals = [fr_leg_points[i][0], fr_leg_points[i+1][0]]
        y_vals = [fr_leg_points[i][1], fr_leg_points[i+1][1]]
        z_vals = [fr_leg_points[i][2], fr_leg_points[i+1][2]]
        lines.append(ax.plot(x_vals, y_vals, z_vals,color=plt_colors[i])[0])


def update_lines(num, coord_data, lines):

    for line, i in zip(lines, range(len(lines))):


        if i < 4:
            # First four lines are the square body
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
             
            x_vals = [coord_data[num][ind][0], coord_data[num][ind+1][0]]
            y_vals = [coord_data[num][ind][1], coord_data[num][ind+1][1]]
            z_vals = [coord_data[num][ind][2], coord_data[num][ind+1][2]]

            line.set_data_3d(x_vals, y_vals, z_vals)

    return lines

if __name__ == "__main__":
    my_robot = QuadrupedRobot()

    hip_positions = my_robot.getHipPosition()
    print(hip_positions)

    FRPose = my_robot.getFRPose()
    my_fr_Leg = ThreeJointLeg(name="FR", start_pose=FRPose)
    fr_leg_points = my_fr_Leg.getLegPoints()
    print(fr_leg_points)

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

    lines = getBodyLine(hip_positions, ax)
    lines = getFRLegLine(fr_leg_points, ax)
    # coord_data = getPsiCoordData(25, -30, 30, )

    # num_angles = 25
    # pitch_angles = np.linspace(-30*d2r,30*d2r,num_angles)
    # coord_data = []
    # for theta in pitch_angles:
    #     # Set a pitch angle
    #     my_robot.setCOMOrientation(theta=theta)

    #     # Get leg coordinates and append to data list
    #     coord_data.append(my_robot.getHipPosition())

    #     my_robot.resetCOMOrientation()


    # coord_data = coord_data + coord_data[::-1]

    # line_ani = animation.FuncAnimation(
    #     fig,
    #     update_lines,
    #     num_angles * 2,
    #     fargs=(coord_data, lines),
    #     interval=75,
    #     blit=False,
    # )

    plt.show()

    print(hip_positions)
