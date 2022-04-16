#! /usr/bin/env python3

from ctypes import *

lib_name = "libEposCmd.so"
epos_lib = cdll.LoadLibrary(lib_name)

class EposMotorDriver:
    pErrorCode = c_uint()
    baudrate = 1000000
    timeout = 500
    acceleration = 0
    deceleration = 0
    def __init__(self, nodeID) -> None:      
        self.nodeID = nodeID
        self.keyHandle = epos_lib.VCS_OpenDevice(b'EPOS4', b'MAXON SERIAL V2', b'USB', b'USB3', byref(self.pErrorCode))
 
        self.enable()
        self.set_velocity_mode()

    def enable(self) -> None:
        epos_lib.VCS_SetProtocolStackSettings(self.keyHandle, self.baudrate, self.timeout, byref(self.pErrorCode))
        epos_lib.VCS_ClearFault(self.keyHandle, self.nodeID, byref(self.pErrorCode))
        epos_lib.VCS_SetEnableState(self.keyHandle, self.nodeID, byref(self.pErrorCode))
        
    def set_velocity_mode(self) -> None:
        epos_lib.VCS_ActivateProfileVelocityMode(self.keyHandle, self.nodeID, byref(self.pErrorCode))
        epos_lib.VCS_SetVelocityProfile(self.keyHandle, self.nodeID, self.acceleration, self.deceleration, byref(self.pErrorCode))

    def disable(self) -> None:
        epos_lib.VCS_SetDisableState(self.keyHandle, self.nodeID, byref(self.pErrorCode))

    def get_avarage_velocity(self) -> int:
        pVelocityIsAveraged = c_long()
        pErrorCode = c_uint()
        epos_lib.VCS_GetVelocityIsAveraged(self.keyHandle, self.nodeID, byref(pVelocityIsAveraged), byref(pErrorCode))
        return pVelocityIsAveraged.value

    def get_avarage_current(self) -> int:
        pCurrentIsAveraged = c_short()
        pErrorCode = c_uint()
        epos_lib.VCS_GetCurrentIsAveraged(self.keyHandle, self.nodeID, byref(pCurrentIsAveraged), byref(pErrorCode))
        return pCurrentIsAveraged.value

class EposSubMotorDriver(EposMotorDriver):
    
    def __init__(self, nodeID, keyHandle0) -> None:
        self.nodeID = nodeID
        self.keyHandle = epos_lib.VCS_OpenSubDevice(keyHandle0 , b'EPOS4', b'CANopen', byref(self.pErrorCode))

        self.enable()
        self.set_velocity_mode()
