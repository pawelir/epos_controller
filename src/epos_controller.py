#! /usr/bin/env python3

from ctypes import *
from platform import node
from epos_motor_driver import EposMotorDriver, EposSubMotorDriver, epos_lib

class EposController:
    def __init__(self) -> None:
        self.motor1 = EposMotorDriver(nodeID=1)
        self.motor2 = EposSubMotorDriver(nodeID=2)
        self.motor3 = EposSubMotorDriver(nodeID=3)
        self.motor4 = EposSubMotorDriver(nodeID=4)
        self.left_side_motors = (self.motor1, self.motor3)
        self.right_side_motors = (self.motor2, self.motor4)
        self.motors = self.left_side_motors + self.right_side_motors

    def move(self, wheel_speed_left=0, wheel_speed_right=0) -> None:
        for motor in self.left_side_motors:
            epos_lib.VCS_MoveWithVelocity(motor.keyHandle, motor.nodeID, wheel_speed_left, byref(motor.pErrorCode))
        for motor in self.right_side_motors:
            epos_lib.VCS_MoveWithVelocity(motor.keyHandle, motor.nodeID, -wheel_speed_right, byref(motor.pErrorCode))

    def stop(self) -> None:
        for motor in self.motors:
            epos_lib.VCS_HaltVelocityMovement(motor.keyHandle, motor.nodeID, byref(motor.pErrorCode))

    def disable_motors(self) -> None:
        for motor in self.motors:
            motor.disable()
        epos_lib.VCS_CloseAllDevices(byref(self.motor1.pErrorCode))
