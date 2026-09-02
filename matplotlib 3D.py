import math
import numpy
import matplotlib.pyplot as plt

def dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return numpy.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])
def forward_kinematics(theta1_deg, theta2_deg, theta3_deg):
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    theta3 = math.radians(theta3_deg)

    T01 = dh_transform(theta1, 0, 0, math.pi / 2)
    T12 = dh_transform(theta2, 0, 0.5, 0)
    T23 = dh_transform(theta3, 0, 0.3, 0)
    T02 = T01 @ T12
    T03 = T02 @ T23

    P0 = numpy.array([0, 0, 0])
    P1 = T01[0:3, 3]
    P2 = T02[0:3, 3]
    P3 = T03[0:3, 3]
    return numpy.array([P0, P1, P2, P3])
def draw_robot(ax, position, R_P0, title):
    x = position[:, 0]
    y = position[:, 1]
    z = position[:, 2]

    ax.plot3D(x, y, z, "b-")
    ax.scatter(x, y, z, color="yellow", s=50)
    ax.scatter(R_P0[0], R_P0[1], R_P0[2], marker="*", color="green", s=150)

    ax.text(x[0], y[0], z[0], "P0")
    ax.text(x[1], y[1], z[1], "P1")
    ax.text(x[2], y[2], z[2], "P2")
    ax.text(x[3], y[3], z[3], "P3")
    ax.text(R_P0[0], R_P0[1], R_P0[2], "R_P0")

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-0.8, 0.8)
    ax.set_ylim(-0.8, 0.8)
    ax.set_zlim(-0.8, 0.8)
theta1 = float(input("theta1: "))
theta2 = float(input("theta2: "))
theta3 = float(input("theta3: "))
if theta1 < -180 or theta1 > 180:
    print("theta1 범위가 넘어갔습니다.")
if theta2 < -90 or theta2 > 90:
    print("theta2 범위가 넘어갔습니다.")
if theta3 < -90 or theta3 > 90:
    print("theta3 범위가 넘어갔습니다.")
if -180 <= theta1 <= 180 and -90 <= theta2 <= 90 and -90 <= theta3 <= 90:
    initial_position = forward_kinematics(0, 0, 0)
    result_position = forward_kinematics(theta1, theta2, theta3)
    R_P0 = result_position[3]

    fig = plt.figure(figsize=(12, 6))

    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    draw_robot(ax1, initial_position, R_P0, "theta = 0, 0, 0")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    draw_robot(ax2, result_position, R_P0, "theta1, theta2, theta3")

    plt.tight_layout()
    plt.show()