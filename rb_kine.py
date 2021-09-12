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
        start_pose,
        start_position={"x": 0, "y": 0, "z": 0},
        link_length={"l1": 0.0838, "l2": 0.2, "l3": 0.2},
        joint_angles={"q1": 0, "q2": 30, "q3": 30},
    ):
        super().__init__()

        self.setHipPosition(start_position)
        self.setLegLength(link_length)
        self.setLegJoints(joint_angles)

    def setHipPosition(self, position):

        self.x = position["x"]
        self.y = position["y"]
        self.z = position["z"]

    def setLegJoints(self, angles):

        self.q1 = angles["q1"]
        self.q2 = angles["q2"]
        self.q3 = angles["q3"]

    def setLegLength(self, lengths):

        self.l1 = lengths["l1"]
        self.l2 = lengths["l2"]
        self.l3 = lengths["l3"]

    def getLegPosition(self):
        pass

    def setLegPosition(self):
        pass

    def getLegPoints(self):
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

        # self.originPoseMat = self.Roll @ self.Pitch @ self.Yaw @ self.originPoseMat
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
    # coord_data = getPsiCoordData(25, -30, 30, )

    num_angles = 25
    pitch_angles = np.linspace(-30*d2r,30*d2r,num_angles)
    coord_data = []
    for theta in pitch_angles:
        # Set a pitch angle
        my_robot.setCOMOrientation(theta=theta)

        # Get leg coordinates and append to data list
        coord_data.append(my_robot.getHipPosition())

        my_robot.resetCOMOrientation()


    coord_data = coord_data + coord_data[::-1]

    line_ani = animation.FuncAnimation(
        fig,
        update_lines,
        num_angles * 2,
        fargs=(coord_data, lines),
        interval=75,
        blit=False,
    )

    plt.show()

    print(hip_positions)
