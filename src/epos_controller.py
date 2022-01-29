#! /usr/bin/env python3

from ctypes import *
from epos_motor_driver import EposMotorDriver, EposSubMotorDriver, epos_lib

class EposController:
    def __init__(self) -> None:
        self.motor1 = EposMotorDriver(1)
        self.motor2 = EposSubMotorDriver(2)
        self.motor3 = EposSubMotorDriver(3)
        self.motor4 = EposSubMotorDriver(4)

    def move(self, Vl=0, Vr=0):
        epos_lib.VCS_MoveWithVelocity(self.motor1.keyHandle, self.motor1.nodeID, Vl, byref(self.motor1.pErrorCode)) #front left
        epos_lib.VCS_MoveWithVelocity(self.motor2.keyHandle, self.motor2.nodeID, -Vr, byref(self.motor2.pErrorCode)) #front right
        epos_lib.VCS_MoveWithVelocity(self.motor3.keyHandle, self.motor3.nodeID, Vl, byref(self.motor3.pErrorCode)) #rear left
        epos_lib.VCS_MoveWithVelocity(self.motor4.keyHandle, self.motor4.nodeID, -Vr, byref(self.motor4.pErrorCode)) #rear right

    def stop(self):
        epos_lib.VCS_HaltVelocityMovement(self.motor1.keyHandle, self.motor1.nodeID, byref(self.motor1.pErrorCode))
        epos_lib.VCS_HaltVelocityMovement(self.motor2.keyHandle, self.motor2.nodeID, byref(self.motor2.pErrorCode))
        epos_lib.VCS_HaltVelocityMovement(self.motor3.keyHandle, self.motor3.nodeID, byref(self.motor3.pErrorCode))
        epos_lib.VCS_HaltVelocityMovement(self.motor4.keyHandle, self.motor4.nodeID, byref(self.motor4.pErrorCode))

    def disable_motors(self):
        self.motor1.disable()
        self.motor2.disable()
        self.motor3.disable()
        self.motor4.disable()
        epos_lib.VCS_CloseAllDevices(byref(self.motor1.pErrorCode))
