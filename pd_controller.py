import math
import time

Kp = 0.007
Ki = 0.00008
Kd = 0.012

MAX_TILT = 0.075
H = 69.5

INTEGRAL_LIMIT = 500


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_vector(nx, ny, nz): #vector has to add up to 1. So, devide everything by the total (to get like a precentage) and then they will add up to 100% (or 1)
    mag = math.sqrt(nx**2 + ny**2 + nz**2)
    return nx / mag, ny / mag, nz / mag


class PDController:
    def __init__(self):
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_time = time.time()

        self.integral_x = 0
        self.integral_y = 0

    def pd_to_normal(self, error_x, error_y):
        now = time.time()
        dt = now - self.last_time #difference in time

        if dt <= 0: 
            dt = 0.001

        # Derivative:
        # Dont actually have to use limits or anything complex
        #Just a linear function so just compute rate of change
        derivative_x = (error_x - self.last_error_x) / dt #change in error x over time (speed of change)
        derivative_y = (error_y - self.last_error_y) / dt

        # Integral:
        #Also not too crazy. This is the total error over a given time. So, multiply total time by error
        self.integral_x += error_x * dt 
        self.integral_y += error_y * dt

        # Anti-windup
        self.integral_x = clamp(
            self.integral_x,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT
        )

        self.integral_y = clamp(
            self.integral_y,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT
        )

        # Add all the x and y in PID
        #Kp is the main driving force. The others make it smoother / more aggresive: 
        out_x = (
            Kp * error_x
            + Ki * self.integral_x
            + Kd * derivative_x
        )

        out_y = (
            Kp * error_y
            + Ki * self.integral_y
            + Kd * derivative_y
        )

        # Limit commanded tilt
        out_x = clamp(out_x, -MAX_TILT, MAX_TILT)
        out_y = clamp(out_y, -MAX_TILT, MAX_TILT)

        #Make negative because camera x and y are opposite of plates
        nx = -out_y
        ny = -out_x
        nz = 1.0

        nx, ny, nz = normalize_vector(nx, ny, nz)

        # Save state
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_time = now

        return nx, ny, nz, H, out_x, out_y