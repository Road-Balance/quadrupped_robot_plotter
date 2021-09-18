import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from QuadrupedKinematics import QuadrupedRobot
from draw_traj import TrajMaker

from math import pi

d2r = pi/180
r2d = 180/pi

points = [
        [-0.1,  0],
        [-0.1, 0.05],
        [-0.1, 0.1],
        [0.1,  0.1],
        [0.1,  0.05],
        [0.1,  0],
    ]

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

    height = 0.3
    
    # IK의 기준은 몸통 끝이다.
    desired_p4_points = {
        "FR": [ 0, -my_robot.hip_length, -height ],
        "FL": [ 0,  my_robot.hip_length, -height ],
        "RR": [ 0, -my_robot.hip_length, -height ],
        "RL": [ 0, my_robot.hip_length, -height ],
    }

    my_robot.setLegPoints(desired_p4_points)

    traj_maker = TrajMaker(
        swing_len=25, points=points, 
        stance_len=25, delta=0.02, L=0.2,
        z_offset=-height
    )

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    hip_positions = my_robot.getBodyPoints()
    leg_points = my_robot.getLegPoints()

    body_lines = getBodyLine(hip_positions, ax)
    leg_lines = getLegLines(leg_points, ax)
    whole_lines = body_lines + leg_lines
    

    # X and Y swap
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_xlim3d([-0.4, 0.4])
    ax.set_ylim3d([-0.2, 0.2])
    ax.set_zlim3d([-0.3, 0.2])
    
    FR_traj = traj_maker.get3DTrajPoints(
            y_offset=-my_robot.hip_length,
            z_offset=-height, phase=1
            )

    FL_traj = traj_maker.get3DTrajPoints(
            y_offset=my_robot.hip_length,
            z_offset=-height, phase=2
            )

    RR_traj = traj_maker.get3DTrajPoints(
            y_offset=-my_robot.hip_length,
            z_offset=-height, phase=2
            )

    RL_traj = traj_maker.get3DTrajPoints(
            y_offset=my_robot.hip_length,
            z_offset=-height, phase=1
            )

    coord_data = []
    for i in range(50):
        p4_points = {
            "FR": FR_traj[i],
            "FL": FL_traj[i],
            "RR": RR_traj[i],
            "RL": RL_traj[i],
        }
        
        my_robot.setLegPoints(p4_points)
        coord_data.append(my_robot.getLegPoints())

    line_ani = animation.FuncAnimation(
        fig,
        update_lines,
        25 * 2,
        fargs=(coord_data, whole_lines),
        interval=20,
        blit=False,
    )

    plt.show()
