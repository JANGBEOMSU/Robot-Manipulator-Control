import math
import numpy

R_P0 = numpy.array(list(map(float, input("R_P0: ").split())))
px = R_P0[0]
py = R_P0[1]
pz = R_P0[2]
def inverse_kinematics_analytical(px, py, pz):
    d = 0
    L2 = 0.5
    L3 = 0.3
    x = px
    y = py
    z = pz

    theta1 = math.atan2(y, x)
    cos_theta3 = (x ** 2 + y ** 2 + (z - d) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
    if cos_theta3 < -1 or cos_theta3 > 1:
        print("목표 위치가 로봇의 작업공간 밖에 있습니다.")
        return None
    sin_theta3_1 = +math.sqrt(1 - cos_theta3 ** 2)
    sin_theta3_2 = -math.sqrt(1 - cos_theta3 ** 2)

    theta3_1 = math.atan2(sin_theta3_1, cos_theta3)
    theta3_2 = math.atan2(sin_theta3_2, cos_theta3)

    theta2_1 = math.atan2(z - d, math.sqrt(x ** 2 + y ** 2)) - math.atan2(L3 * sin_theta3_1, L2 + L3 * cos_theta3)
    theta2_2 = math.atan2(z - d, math.sqrt(x ** 2 + y ** 2)) - math.atan2(L3 * sin_theta3_2, L2 + L3 * cos_theta3)

    theta1_deg = math.degrees(theta1)
    theta2_1_deg = math.degrees(theta2_1)
    theta2_2_deg = math.degrees(theta2_2)
    theta3_1_deg = math.degrees(theta3_1)
    theta3_2_deg = math.degrees(theta3_2)

    position1 = [theta1_deg, theta2_1_deg, theta3_1_deg]
    position2 = [theta1_deg, theta2_2_deg, theta3_2_deg]

    valid_positions = []
    if -180 <= theta1_deg <= 180 and -90 <= theta2_1_deg <= 90 and -90 <= theta3_1_deg <= 90:
        valid_positions.append(position1)
    if -180 <= theta1_deg <= 180 and -90 <= theta2_2_deg <= 90 and -90 <= theta3_2_deg <= 90:
        valid_positions.append(position2)
    if len(valid_positions) == 0:
        print("관절 범위 안에 들어오는 유효한 해가 없습니다.")
        return None
    posture_positions = []
    for position in valid_positions:
        if position[2] > 0:
            posture_positions.append(["elbow up", numpy.array(position)])
        else:
            posture_positions.append(["elbow down", numpy.array(position)])
    return posture_positions
result = inverse_kinematics_analytical(px, py, pz)
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

    T01 = dh_transform(theta1,0,0,math.pi/2)
    T12 = dh_transform(theta2,0,0.5,0)
    T23 = dh_transform(theta3,0,0.3,0)
    T03 = T01 @ T12 @ T23
    return T03
if result is not None:
    for posture_name, position in result:
        T03 = forward_kinematics(position[0], position[1], position[2])
        FK_position = T03 @ numpy.array([0, 0, 0, 1])
        error = numpy.linalg.norm(R_P0[0:3] - FK_position[0:3])
        if posture_name == "elbow down":
            print("팔꿈치 아래 자세")
        elif posture_name == "elbow up":
            print("팔꿈치 위 자세")
        else:
            print("유효한 해")
        print("theta1, theta2, theta3:","theta1:", position[0], ", theta2 =", position[1], ", theta3 =", position[2])
        print("FK로 계산한 위치:","엔드 이팩터 x=", FK_position[0], ", 엔드 이팩터 y =", FK_position[1], ", 엔드 이팩터 z =", FK_position[2])
        print("목표 위치와의 오차:",(error))
    if len(result) > 1:
      for posture_name, position in result:
        if posture_name == "elbow up":
            print("최종 선택 해 = 팔꿈치 위 자세")
            print("theta1, theta2, theta3:","theta1 =", position[0], ", theta2 =", position[1], ", theta3 =", position[2])