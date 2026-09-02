import numpy

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

print("R_T_O 1행 =", R_T_O[0][0], ",", R_T_O[0][1], ",", R_T_O[0][2], ",", R_T_O[0][3])
print("R_T_O 2행 =", R_T_O[1][0], ",", R_T_O[1][1], ",", R_T_O[1][2], ",", R_T_O[1][3])
print("R_T_O 3행 =", R_T_O[2][0], ",", R_T_O[2][1], ",", R_T_O[2][2], ",", R_T_O[2][3])
print("R_T_O 4행 =", R_T_O[3][0], ",", R_T_O[3][1], ",", R_T_O[3][2], ",", R_T_O[3][3])
print("R_P0 =", R_P0[0], ",", R_P0[1], ",", R_P0[2], ",", R_P0[3])