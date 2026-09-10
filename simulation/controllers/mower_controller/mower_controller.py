"""Webots mower controller."""
from controller import Robot


robot = Robot()

TIME_STEP = 16

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(TIME_STEP) != -1:
    pass