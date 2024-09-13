import numpy as np
import math

# Constants
_PI = math.pi
_PI_2 = 0.5 * _PI
_2_PI = 2.0 * _PI

# TM700 DH parameters
d1 = 0.1451
a2 = 0.3290
a3 = 0.3115
d4 = -0.1222
d5 = 0.1060
d6 = 0.1144+0.260

# Tolerances
IkDistTOL = 1e-100
IkAngTOL = 1e-100

def inverse_q1(T):
    L0J1J6 = np.zeros(3)
    q1 = np.zeros(2)
    for i in range(3):
        L0J1J6[i] = T[i, 3] - T[i, 2] * d6
    L0J1J6[2] -= d1
    r = math.sqrt(L0J1J6[0] ** 2 + L0J1J6[1] ** 2)
    if r > -d4 + IkDistTOL:
        asp = math.asin(-d4 / r)
        q1[0] = math.atan2(L0J1J6[1], L0J1J6[0]) + asp
        q1[1] = math.atan2(-L0J1J6[1], -L0J1J6[0]) - asp
        num_sols_q1 = 2
    elif -d4 - IkDistTOL <= r <= -d4 + IkDistTOL:
        q1[0] = math.atan2(L0J1J6[1], L0J1J6[0]) + _PI_2
        q1[1] = math.atan2(-L0J1J6[1], -L0J1J6[0]) - _PI_2
        num_sols_q1 = -1
    else:
        num_sols_q1 = 0
    if num_sols_q1 != 0:
        # q1 in [-180, +180]
        for i in range(2):
            if q1[i] > _PI - math.radians(2):
                 q1[i] -= _2_PI
            if q1[i] < -_PI + math.radians(2):
                q1[i] += _2_PI
    return num_sols_q1, q1, L0J1J6

def inverse_qp56(T, q1, q_ref, isRA):
    R10 = np.zeros((3, 3))
    R16 = np.zeros((3, 3))
    qp56 = np.zeros(6)
    R10[0, 0] = math.cos(q1)
    R10[0, 1] = math.sin(q1)
    R10[1, 2] = -1
    R10[2, 0] = -R10[0, 1]
    R10[2, 1] = R10[0, 0]
    for i in range(3):
        for j in range(3):
            R16[i, j] = sum(R10[i, k] * T[k, j] for k in range(3))
    if R16[2, 2] <= -1 + IkAngTOL:
        qp56[1] = 0
        num_sols_qp56 = -1
    elif R16[2, 2] >= 1 - IkAngTOL:
        qp56[1] = _PI
        num_sols_qp56 = -1
    else:
        if isRA:
            qp56[1] = math.acos(-R16[2, 2])
        else:
            qp56[1] = -math.acos(-R16[2, 2])
        qp56[4] = -qp56[1]
        num_sols_qp56 = 2
    if num_sols_qp56 == 2:
        if isRA:
            qp56[0] = math.atan2(R16[1, 2], R16[0, 2])
            qp56[3] = math.atan2(-R16[1, 2], -R16[0, 2])
            qp56[2] = math.atan2(-R16[2, 1], R16[2, 0])
            qp56[5] = math.atan2(R16[2, 1], -R16[2, 0])
        else:
            qp56[0] = math.atan2(-R16[1, 2], -R16[0, 2])
            qp56[3] = math.atan2(R16[1, 2], R16[0, 2])
            qp56[2] = math.atan2(R16[2, 1], -R16[2, 0])
            qp56[5] = math.atan2(-R16[2, 1], R16[2, 0])
    else:
        qp56[2] = q_ref[5]
        cq = math.cos(qp56[2])
        sq = math.sin(qp56[2])
        qp56[0] = math.atan2(sq * R16[0, 0] + R16[0, 2] * cq, R16[0, 0] * cq - sq * R16[0, 2])
        qp56[3] = qp56[0]
        qp56[5] = qp56[2]
        num_sols_qp56 = 1
    return num_sols_qp56, qp56, R10, R16

def inverse_q234(T, L0J1J6, R10, qp, q_ref, isRA):
    L1J1J5 = np.zeros(3)
    q234 = np.zeros(6)
    for i in range(3):
        L1J1J5[i] = sum(R10[i, j] * L0J1J6[j] for j in range(3))
    L1J1J5[0] -= math.sin(qp) * d5
    L1J1J5[1] += math.cos(qp) * d5
    r = math.sqrt(L1J1J5[0] ** 2 + L1J1J5[1] ** 2)
    s = a2 + a3 - r
    t = r - abs(a2 - a3)
    if s >= IkDistTOL and t >= IkDistTOL:
        atp = math.atan2(L1J1J5[1], L1J1J5[0])
        acp1 = math.acos((a2 ** 2 + r ** 2 - a3 ** 2) / (2.0 * a2 * r))
        acp2 = math.acos((a2 ** 2 + a3 ** 2 - r ** 2) / (2.0 * a2 * a3))
        if isRA:
            q234[0] = atp - acp1
            q234[3] = atp + acp1
            q234[1] = _PI - acp2
            q234[4] = acp2 - _PI
        else:
            q234[0] = atp + acp1
            q234[3] = atp - acp1
            q234[1] = acp2 - _PI
            q234[4] = _PI - acp2
        for i in range(2):
            isol = 3 * i
            # q2 in [-180, +180]
            if q234[isol] > _PI - math.radians(2):
                q234[isol] -= _2_PI
            elif q234[isol] < -_PI + math.radians(2):
                q234[isol] += _2_PI
        num_sols_q234 = 2
    elif -IkDistTOL < s < IkDistTOL:
        q234[0] = math.atan2(L1J1J5[1], L1J1J5[0])
        q234[3] = q234[0]
        q234[1] = 0
        q234[4] = 0
        num_sols_q234 = 1
    elif -IkDistTOL < t < IkDistTOL:
        q234[0] = math.atan2(L1J1J5[1], L1J1J5[0])
        q234[3] = q234[0]
        if q_ref[2] > 0:
            q234[1] = _PI
            q234[4] = _PI
            num_sols_q234 = 1
        elif q_ref[2] < 0:
            q234[1] = -_PI
            q234[4] = -_PI
            num_sols_q234 = 1
        else:
            if isRA:
                q234[1] = _PI
                q234[4] = -_PI
            else:
                q234[1] = -_PI
                q234[4] = _PI
            num_sols_q234 = 2
    else:
        num_sols_q234 = 0
    if num_sols_q234 > 0:
        for i in range(2):
            isol = 3 * i
            q234[2 + isol] = qp - q234[isol] - q234[1 + isol]
            # q4 in [-180, +180]
            if q234[2 + isol] > _PI - math.radians(2):
                q234[2 + isol] -= _2_PI
            elif q234[2 + isol] < -_PI + math.radians(2):
                q234[2 + isol] += _2_PI
    return num_sols_q234, q234

def forward(q):
    T = np.zeros((4, 4))
    c1, c2, c3, c4, c5, c6 = [math.cos(angle) for angle in q]
    s1, s2, s3, s4, s5, s6 = [math.sin(angle) for angle in q]
    c2, s2 = math.cos(q[1] - _PI_2), math.sin(q[1] - _PI_2)
    c4, s4 = math.cos(q[3] + _PI_2), math.sin(q[3] + _PI_2)
    cp, sp = math.cos(q[1] + q[2] + q[3]), math.sin(q[1] + q[2] + q[3])

    T[0, 0] = c1 * sp * s6 - s1 * s5 * c6 + c1 * cp * c5 * c6
    T[0, 1] = c1 * sp * c6 + s1 * s5 * s6 - c1 * cp * c5 * s6
    T[0, 2] = c1 * cp * s5 + s1 * c5
    T[0, 3] = (c1 * (a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3) - d4 * s1 +
               d6 * c5 * s1 + d5 * c1 * (c4 * (c2 * s3 + c3 * s2) +
               s4 * (c2 * c3 - s2 * s3)) + d6 * c1 * s5 * (c4 * (c2 * c3 - s2 * s3) -
               s4 * (c2 * s3 + c3 * s2)))

    T[1, 0] = s1 * sp * s6 + c1 * s5 * c6 + s1 * cp * c5 * c6
    T[1, 1] = s1 * sp * c6 - c1 * s5 * s6 - s1 * cp * c5 * s6
    T[1, 2] = s1 * cp * s5 - c1 * c5
    T[1, 3] = (s1 * (a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3) + d4 * c1 -
               d6 * c1 * c5 + d5 * s1 * (c4 * (c2 * s3 + c3 * s2) +
               s4 * (c2 * c3 - s2 * s3)) + d6 * s1 * s5 * (c4 * (c2 * c3 - s2 * s3) -
               s4 * (c2 * s3 + c3 * s2)))

    T[2, 0] = cp * s6 - sp * c5 * c6
    T[2, 1] = cp * c6 + sp * c5 * s6
    T[2, 2] = -sp * s5
    T[2, 3] = (d1 - a2 * s2 + d5 * (c4 * (c2 * c3 - s2 * s3) -
               s4 * (c2 * s3 + c3 * s2)) - a3 * c2 * s3 - a3 * c3 * s2 -
               d6 * s5 * (c4 * (c2 * s3 + c3 * s2) + s4 * (c2 * c3 - s2 * s3)))

    T[3, 3] = 1
    return T

def inverse(position, Rx, Ry, Rz, q_ref=None):
    Rx_matrix = [[1, 0, 0, 0],
                 [0, math.cos(math.radians(Rx)), -math.sin(math.radians(Rx)), 0],
                 [0, math.sin(math.radians(Rx)), math.cos(math.radians(Rx)), 0],
                 [0, 0, 0, 1]]

    Ry_matrix = [[math.cos(math.radians(Ry)), 0, math.sin(math.radians(Ry)), 0],
                 [0, 1, 0, 0],
                 [-math.sin(math.radians(Ry)), 0, math.cos(math.radians(Ry)), 0],
                 [0, 0, 0, 1]]

    Rz_matrix = [[math.cos(math.radians(Rz)), -math.sin(math.radians(Rz)), 0, 0],
                 [math.sin(math.radians(Rz)), math.cos(math.radians(Rz)), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]
    T = np.dot(Rz_matrix, np.dot(Ry_matrix, Rx_matrix))
    T[0, 3] = position[0]
    T[1, 3] = position[1]
    T[2, 3] = position[2]
    num_sols = 0
    q_sols = []
    if q_ref is None:
        q_ref = [0, -_PI_2, 0, _PI_2, 0, 0]
    else:
        q_ref = list(q_ref)
    q_ref[1] -= _PI_2
    q_ref[3] += _PI_2
    for i in range(6):
        if q_ref[i] > _PI:
            q_ref[i] -= _2_PI
        elif q_ref[i] < -_PI:
            q_ref[i] += _2_PI

    num_sols_q1, q1, L0J1J6 = inverse_q1(T)
    if num_sols_q1 > 0:
        for i in range(num_sols_q1):
            num_sols_qp56, qp56, R10, R16 = inverse_qp56(T, q1[i], q_ref, (i == 0))
            if num_sols_qp56 > 0:
                for j in range(num_sols_qp56):
                    num_sols_q234, q234 = inverse_q234(T, L0J1J6, R10, qp56[3 * j], q_ref, (i == 0))
                    if num_sols_q234 > 0:
                        for k in range(num_sols_q234):
                            q_sol = [0] * 6
                            q_sol[0] = q1[i]
                            q_sol[1] = q234[3 * k] + _PI_2
                            # q2 in [-180, +180]
                            if q_sol[1] > _PI - math.radians(1):
                                q_sol[1] -= _2_PI
                            elif q_sol[1] < -_PI + math.radians(1):
                                q_sol[1] += _2_PI
                            q_sol[2] = q234[1 + 3 * k]
                            q_sol[3] = q234[2 + 3 * k] - _PI_2
                            # q4 in [-180, +180]
                            if q_sol[3] > _PI - math.radians(1):
                                q_sol[3] -= _2_PI
                            elif q_sol[3] < -_PI + math.radians(1):
                                q_sol[3] += _2_PI
                            q_sol[4] = qp56[1 + 3 * j]
                            q_sol[5] = qp56[2 + 3 * j]
                            q_sols.append(q_sol)
                            num_sols += 1
    return num_sols, np.degrees(q_sols)

if __name__ == "__main__":
    # Test the inverse kinematics with a specific position and orientation
    position = [0.458745, 0.1223, 0.756169]
    Rx = 80
    Ry = 0
    Rz = 90

    # Transformation matricies (4x4) [R, t], [0,1]
    Rx_matrix = [[1, 0, 0, 0],
                 [0, math.cos(math.radians(Rx)), -math.sin(math.radians(Rx)), 0],
                 [0, math.sin(math.radians(Rx)), math.cos(math.radians(Rx)), 0],
                 [0, 0, 0, 1]]

    Ry_matrix = [[math.cos(math.radians(Ry)), 0, math.sin(math.radians(Ry)), 0],
                 [0, 1, 0, 0],
                 [-math.sin(math.radians(Ry)), 0, math.cos(math.radians(Ry)), 0],
                 [0, 0, 0, 1]]

    Rz_matrix = [[math.cos(math.radians(Rz)), -math.sin(math.radians(Rz)), 0, 0],
                 [math.sin(math.radians(Rz)), math.cos(math.radians(Rz)), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]

    # Compute the transformation matrix
    T = np.dot(Rz_matrix, np.dot(Ry_matrix, Rx_matrix))
    T[0, 3] = position[0]
    T[1, 3] = position[1]
    T[2, 3] = position[2]

    # Call the inverse kinematics function
    num_sols, q_sols = inverse(position, Rx, Ry, Rz)
    print(f"Number of solutions: {num_sols}")
    for i, q in enumerate(q_sols):
        print(f"Solution {i + 1}: {q}")
    print(len(q_sols))
    #T = forward([radians(180), radians(30), radians(-90), radians(70), radians(-90), radians(0)])
    #print(T)