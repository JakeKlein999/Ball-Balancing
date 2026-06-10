import math

# Geometry in mm
F = 62.0     # servo arm length
G = 76.2     # connecting rod length

E = 87.4     # platform joint radius
D = 48.4     # servo pivot radius

HOME_H = 70.0


def clamp(x, low, high):
    return max(low, min(high, x))


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def sub(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]


def mat_vec_mul(M, v):
    return [
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
    ]


def rotation_matrix_from_normal(nx, ny, nz):
    length = math.sqrt(nx*nx + ny*ny + nz*nz)

    if length == 0:
        raise ValueError("Normal vector cannot be zero.")

    nx /= length
    ny /= length
    nz /= length

    if abs(1 + nz) < 1e-6:
        raise ValueError("Normal vector too close to straight down.")

    return [
        [1 - nx*nx / (1 + nz), -nx*ny / (1 + nz), nx],
        [-nx*ny / (1 + nz), 1 - ny*ny / (1 + nz), ny],
        [-nx, -ny, nz]
    ]


def solve_leg(platform_point, base_point, leg_direction, R, h):
    P = mat_vec_mul(R, platform_point)
    P[2] += h

    v = sub(P, base_point)

    horizontal = dot(v, leg_direction)
    vertical = v[2]

    m = math.sqrt(horizontal*horizontal + vertical*vertical)

    if m < 1e-6:
        raise ValueError("Bad geometry.")

    beta_arg = (m*m + F*F - G*G) / (2 * m * F)
    beta_arg = clamp(beta_arg, -1, 1)

    alpha = math.atan2(vertical, horizontal)
    beta = math.acos(beta_arg)

    theta1 = math.degrees(alpha + beta)
    theta2 = math.degrees(alpha - beta)

    theta = theta1 if abs(theta1) < abs(theta2) else theta2

    return theta


def inverse_kinematics(nx, ny, nz, h=HOME_H):
    R = rotation_matrix_from_normal(nx, ny, nz)

    platform_points = {
        "A": [E, 0, 0],
        "B": [-E / 2, math.sqrt(3) * E / 2, 0],
        "C": [-E / 2, -math.sqrt(3) * E / 2, 0]
    }

    base_points = {
        "A": [D, 0, 0],
        "B": [-D / 2, math.sqrt(3) * D / 2, 0],
        "C": [-D / 2, -math.sqrt(3) * D / 2, 0]
    }

    leg_directions = {
        "A": [1, 0, 0],
        "B": [-1 / 2, math.sqrt(3) / 2, 0],
        "C": [-1 / 2, -math.sqrt(3) / 2, 0]
    }

    thetas = {}

    for name in ["A", "B", "C"]:
        thetas[name] = solve_leg(
            platform_points[name],
            base_points[name],
            leg_directions[name],
            R,
            h
        )

    return thetas