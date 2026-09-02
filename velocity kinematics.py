import math
import numpy

R_P0 = numpy.array(list(map(float, input("R_P0: ").split())))
P_target = R_P0[0:3]

theta1 = 0.0
theta2 = 0.0
theta3 = 0.0
q = numpy.array([theta1, theta2, theta3])

def dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return numpy.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])
def forward_kinematics(q):
    T01 = dh_transform(q[0], 0, 0, math.pi / 2)
    T12 = dh_transform(q[1], 0, 0.5, 0)
    T23 = dh_transform(q[2], 0, 0.3, 0)
    T02 = T01 @ T12
    T03 = T02 @ T23
    return T01, T02, T03
def jacobian(q):
    T01, T02, T03 = forward_kinematics(q)
    p1 = numpy.array([0, 0, 0])
    p2 = T01[0:3, 3]
    p3 = T02[0:3, 3]
    p_end = T03[0:3, 3]

    z1 = numpy.array([0, 0, 1])
    z2 = T01[0:3, 2]
    z3 = T02[0:3, 2]

    J1 = numpy.cross(z1, p_end - p1)
    J2 = numpy.cross(z2, p_end - p2)
    J3 = numpy.cross(z3, p_end - p3)
    return numpy.array([J1, J2, J3]).T
def analytical_ik(px, py, pz):
    d = 0
    L2 = 0.5
    L3 = 0.3
    x = px
    y = py
    z = pz

    theta1 = math.atan2(y, x)
    cos_theta3 = (x ** 2 + y ** 2 + (z - d) ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)

    if cos_theta3 < -1 or cos_theta3 > 1:
        return None
    sin_theta3_2 = -math.sqrt(1 - cos_theta3 ** 2)
    theta3_2 = math.atan2(sin_theta3_2, cos_theta3)
    theta2_2 = math.atan2(z - d, math.sqrt(x ** 2 + y ** 2)) - math.atan2(L3 * sin_theta3_2, L2 + L3 * cos_theta3)

    return numpy.array([theta1, theta2_2, theta3_2])
alpha = 0.5
max_iter = 10000
tolerance = 10 ** -4
q_min = numpy.radians(numpy.array([-180, -90, -90]))
q_max = numpy.radians(numpy.array([180, 90, 90]))

for count in range(max_iter):
    T01, T02, T03 = forward_kinematics(q)
    P_current = T03[0:3, 3]
    e = P_target - P_current
    error = math.sqrt(e[0] ** 2 + e[1] ** 2 + e[2] ** 2)
    if error < tolerance:
        break
    J = jacobian(q)
    dtheta = J.T @ e
    q = q + alpha * dtheta
    q = numpy.maximum(q_min, numpy.minimum(q, q_max))
T01, T02, T03 = forward_kinematics(q)
P_velocity = T03[0:3, 3]
velocity_error = numpy.linalg.norm(P_target - P_velocity)

q_analytical = analytical_ik(P_target[0], P_target[1], P_target[2])

print("해석적 IK 결과")
if q_analytical is not None:
    q_analytical_deg = numpy.degrees(q_analytical)
    print("theta1 =", q_analytical_deg[0], ", theta2 =", q_analytical_deg[1], ", theta3 =", q_analytical_deg[2])
    T01_a, T02_a, T03_a = forward_kinematics(q_analytical)
    end_effector = T03_a[0:3, 3]
    analytical_error = numpy.linalg.norm(P_target - end_effector)
    print("해석적 IK 위치 오차 =", analytical_error)
else:
    print("해석적 IK 해가 없습니다.")
q_velocity_deg = numpy.degrees(q)
print("속도 기구학 결과")
print("theta1 =", q_velocity_deg[0], ", theta2 =", q_velocity_deg[1], ", theta3 =", q_velocity_deg[2])
print("속도 기구학 end-effector 위치 =>", "x =", P_velocity[0], ", y =", P_velocity[1], ", z =", P_velocity[2])
print("속도 기구학 위치 오차 =", velocity_error)
print("속도 기구학 수렴 반복 횟수 =", count)