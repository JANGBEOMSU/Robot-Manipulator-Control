import math
import numpy
import matplotlib.pyplot as plt

R_T_C = numpy.array([
    [0, -1, 0, 0.2],
    [1, 0, 0, 0],
    [0, 0, 1, 0.8],
    [0, 0, 0, 1],
])
print("C_T_O 값을 입력하세요.")
C_T_O = numpy.array([
    list(map(float, input("C_T_O 1행: ").split())),
    list(map(float, input("C_T_O 2행: ").split())),
    list(map(float, input("C_T_O 3행: ").split())),
    list(map(float, input("C_T_O 4행: ").split())),
])
print("O_P0 값을 입력하세요.")
O_P0 = numpy.array(list(map(float, input("O_P0: ").split())))
R_T_O = R_T_C @ C_T_O
R_P0 = R_T_O @ O_P0
target = R_P0[0:3]

def inverse_kinematics_analytical(px, py, pz):
    L2 = 0.5
    L3 = 0.3
    xy = math.sqrt(px ** 2 + py ** 2)
    theta1 = math.atan2(py, px)
    cos_theta3 = (xy ** 2 + pz ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
    if cos_theta3 < -1 or cos_theta3 > 1:
        print("목표 위치가 로봇의 작업공간 밖에 있습니다.")
        return None
    sin_theta3_1 = math.sqrt(1 - cos_theta3 ** 2)
    sin_theta3_2 = -math.sqrt(1 - cos_theta3 ** 2)

    theta3_1 = math.atan2(sin_theta3_1, cos_theta3)
    theta3_2 = math.atan2(sin_theta3_2, cos_theta3)

    theta2_1 = math.atan2(pz, xy) - math.atan2(L3 * sin_theta3_1, L2 + L3 * cos_theta3)
    theta2_2 = math.atan2(pz, xy) - math.atan2(L3 * sin_theta3_2, L2 + L3 * cos_theta3)

    theta1_deg = math.degrees(theta1)
    theta2_1_deg = math.degrees(theta2_1)
    theta2_2_deg = math.degrees(theta2_2)
    theta3_1_deg = math.degrees(theta3_1)
    theta3_2_deg = math.degrees(theta3_2)

    solution1 = [theta1_deg, theta2_1_deg, theta3_1_deg]
    solution2 = [theta1_deg, theta2_2_deg, theta3_2_deg]

    valid_solutions = []
    if -180 <= theta1_deg <= 180 and -90 <= theta2_1_deg <= 90 and -90 <= theta3_1_deg <= 90:
        valid_solutions.append(solution1)
    if -180 <= theta1_deg <= 180 and -90 <= theta2_2_deg <= 90 and -90 <= theta3_2_deg <= 90:
        valid_solutions.append(solution2)
    if len(valid_solutions) == 0:
        print("관절 범위 안에 들어오는 유효한 해가 없습니다.")
        return None

    if len(valid_solutions) == 1:
        selected_solution = valid_solutions[0]
    elif abs(valid_solutions[0][1]) >= abs(valid_solutions[1][1]):
        selected_solution = valid_solutions[0]
    else:
        selected_solution = valid_solutions[1]
    return numpy.array(selected_solution)
theta = inverse_kinematics_analytical(target[0], target[1], target[2])

def dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return numpy.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])
def forward_kinematics_positions(theta1_deg, theta2_deg, theta3_deg):
    theta1 = math.radians(theta1_deg)
    theta2 = math.radians(theta2_deg)
    theta3 = math.radians(theta3_deg)

    T01 = dh_transform(theta1, 0, 0, math.pi / 2)
    T12 = dh_transform(theta2, 0, 0.5, 0)
    T23 = dh_transform(theta3, 0, 0.3, 0)
    T02 = T01 @ T12
    T03 = T02 @ T23

    p0 = numpy.array([0, 0, 0])
    p1 = T01[0:3, 3]
    p2 = T02[0:3, 3]
    p3 = T03[0:3, 3]
    return numpy.array([p0, p1, p2, p3])

def draw_robot(ax, positions, target, title):
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]

    ax.plot3D(x, y, z, "b-", linewidth=3)
    ax.scatter(x, y, z, color="yellow", s=50)
    ax.scatter(target[0], target[1], target[2], color="green", marker="*", s=200)

    ax.text(x[0], y[0], z[0], "Base")
    ax.text(x[1], y[1], z[1], "Joint1")
    ax.text(x[2], y[2], z[2], "Joint2")
    ax.text(x[3], y[3], z[3], "End-effector")
    ax.text(target[0], target[1], target[2], "Target")

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([-0.8, 0.8])
    ax.set_ylim([-0.8, 0.8])
    ax.set_zlim([-0.8, 0.8])
if theta is not None:
    initial_positions = forward_kinematics_positions(0, 0, 0)
    ik_positions = forward_kinematics_positions(theta[0], theta[1], theta[2])

    print("R_P0 = ")
    print(R_P0)
    print("시각화에 사용된 theta1, theta2, theta3 = ")
    print(theta)
    fig = plt.figure(figsize=(12, 6))

    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    draw_robot(ax1, initial_positions, target, "basic (0, 0, 0)")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    draw_robot(ax2, ik_positions, target, "IK posture")

    plt.tight_layout()
    plt.show()