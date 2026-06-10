import subprocess
import time
from time import sleep
import math


from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

from camera_tracker import BallTracker
from pd_controller import PDController
from kinematics import inverse_kinematics


# Servo setup
PINS = {
    "A": 12,
    "B": 13,
    "C": 18
}

OFFSETS = {
    "A": 4,
    "B": -8,
    "C": -6
}

DIRECTIONS = {
    "A": 1,
    "B": 1,
    "C": 1
}

SERVO_HORIZONTAL_COMMAND = 90

 
def clamp(x, low, high): #this is used to avoid tilts above the max tilt 
    return max(low, min(high, x))


def start_pigpio():
    result = subprocess.run(
        ["pgrep", "pigpiod"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    if result.returncode != 0:
        print("Starting pigpiod...")
        subprocess.run(["sudo", "/usr/local/bin/pigpiod"], check=False)
        time.sleep(1)
    else:
        print("pigpiod already running.")


def set_servo(servos, name, theta):
    command = SERVO_HORIZONTAL_COMMAND
    command -= DIRECTIONS[name] * theta
    command += OFFSETS[name]
    command = clamp(command, 0, 180)

    servos[name].angle = command
    return command


def set_all_home(servos): #self explanatory
    for name in ["A", "B", "C"]:
        servos[name].angle = SERVO_HORIZONTAL_COMMAND + OFFSETS[name]


# Main
start_pigpio()
factory = PiGPIOFactory()

#this is a more efficient way to assign every servo with the same traits
#We are making a class using the AngularServo library. You cannot see it, but self.angle = 0
#each servo is its own object
servos = { 
    name: AngularServo(
        pin,
        min_angle=0,
        max_angle=180,
        min_pulse_width=0.0005,
        max_pulse_width=0.0025,
        pin_factory=factory
    )
    for name, pin in PINS.items()
}


tracker = BallTracker(show_windows=False) #Object is tracker. Just runs init which will intialize the camera 
controller = PDController() #Creates controler as object. Assigns the attributes like the time, last error, and intergal

# desired_position = [5, 6.5]

PATTERN_POINTS = [
    [5.0, 6.5]     # top  # bottom left
]

point_index = 0
POINT_HOLD_TIME = 6.5
last_switch_time = time.time()


try:
    print("Going home...")
    set_all_home(servos)
    sleep(1)

    print("Starting ball balancer. Press Ctrl+C to stop.")

    while True:
        ball = tracker.get_ball_position()

        if ball is None:
            print("No ball detected")
            set_all_home(servos)
            sleep(0.02)
            continue

        ball_x, ball_y = ball

        now = time.time()

        if now - last_switch_time > POINT_HOLD_TIME:
            point_index = (point_index + 1) % len(PATTERN_POINTS)
            last_switch_time = now

        desired_position = PATTERN_POINTS[point_index]

        error_x = ball_x - desired_position[0]
        error_y = ball_y - desired_position[1]

        #pass error into PDI which will return the desired normal vector for the plate 
        nx, ny, nz, h, out_x, out_y = controller.pd_to_normal(error_x, error_y)

        #Then, pass that desired plate orientation into the IK which will return the theta for each leg
        thetas = inverse_kinematics(nx, ny, nz, h)

        commands = {}

        for name in ["A", "B", "C"]:
            commands[name] = set_servo(servos, name, thetas[name])

        print(
            # f"error=({error_x:.0f},{error_y:.0f}) "
            # f"normal=({nx:.3f},{ny:.3f},{nz:.3f}) "
            # f"theta=({thetas['A']:.1f},{thetas['B']:.1f},{thetas['C']:.1f}) "
            # f"cmd=({commands['A']:.1f},{commands['B']:.1f},{commands['C']:.1f})"
            desired_position
        )

        sleep(0.02)

except KeyboardInterrupt:
    print("\nStopping. Returning home...")
    set_all_home(servos)
    sleep(1)

finally:
    tracker.stop()

    for servo in servos.values():
        servo.detach()

    print("Done.")