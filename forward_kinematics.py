import math
import numpy

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
theta1 = float(input("theta1(디그리 값) 값을 입력하세요: "))
theta2 = float(input("theta2(디그리 값) 값을 입력하세요: "))
theta3 = float(input("theta3(디그리 값) 값을 입력하세요: "))

if theta1 < -180 or theta1 > 180:
    print("theta1 범위가 넘어갔습니다.")
if theta2 < -90 or theta2 > 90:
    print("theta2 범위가 넘어갔습니다.")
if theta3 < -90 or theta3 > 90:
    print("theta3 범위가 넘어갔습니다.")

T03 = forward_kinematics(theta1, theta2, theta3)
print("T03 1행 =", T03[0][0], ",", T03[0][1], ",", T03[0][2], ",", T03[0][3])
print("T03 2행 =", T03[1][0], ",", T03[1][1], ",", T03[1][2], ",", T03[1][3])
print("T03 3행 =", T03[2][0], ",", T03[2][1], ",", T03[2][2], ",", T03[2][3])
print("T03 4행 =", T03[3][0], ",", T03[3][1], ",", T03[3][2], ",", T03[3][3])