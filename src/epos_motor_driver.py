#! /usr/bin/env python3

from ctypes import *
import time
# sprobowac zaladowac biblioteke po nazwie (lib_name) - wtedy laduje sie z domyslnej lokalizacji gdzie linux instaluje libki
# jeśli nie zadziała, wrzucić libke do lokalizacji pliku ze skryptem i użyć lib_path do załadowania
# lib_path = "./libEposCmd.so.6.7.1.0"
lib_name = "libEposCmd.so.6.7.1.0"    
epos_lib = cdll.LoadLibrary(lib_name)

class EposMotorDriver:
    # czy pErrorCode powinien byc per motor czy jeden wspólny?
    # które z poniższych są niepotrzebne?
    pErrorCode = c_uint()

    baudrate = 1000000
    timeout = 500
    # nie powinniśmy od razu ustawić przyśpieszenia na wartosć róźną od zera?
    acceleration = 0
    deceleration = 0

    def __init__(self, nodeID) -> None:      
        self.nodeID = nodeID
        self.keyHandle = epos_lib.VCS_OpenDevice(b'EPOS4', b'MAXON SERIAL V2', b'USB', b'USB3', byref(self.pErrorCode)) # USB3 czy USB0?

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

    def get_avarage_velocity(self):
        pVelocityIsAveraged = c_long()
        pErrorCode = c_uint()
        epos_lib.VCS_GetVelocityIsAveraged(self.keyHandle, self.nodeID, byref(pVelocityIsAveraged), byref(pErrorCode))
        return pVelocityIsAveraged.value

    def get_avarage_current(self):
        pCurrentIsAveraged = c_short()
        pErrorCode = c_uint()
        ret = epos_lib.VCS_GetCurrentIsAveraged(self.keyHandle, self.nodeID, byref(pCurrentIsAveraged), byref(pErrorCode))
        return pCurrentIsAveraged.value


class EposSubMotorDriver(EposMotorDriver):
    
    def __init__(self, nodeID, keyHandle0) -> None:
        self.nodeID = nodeID
        self.keyHandle = epos_lib.VCS_OpenSubDevice(keyHandle0 , b'EPOS4', b'CANopen', byref(self.pErrorCode))

        self.enable()
        self.set_velocity_mode()


if __name__=="__main__":
    motor = EposMotorDriver(1)
    print('motor initialized')
    epos_lib.VCS_MoveWithVelocity(motor.keyHandle, motor.nodeID, 10, byref(motor.pErrorCode))
    print('motor move, waiting 3 sec...')
    time.sleep(3)
    epos_lib.VCS_HaltVelocityMovement(motor.keyHandle, motor.nodeID, byref(motor.pErrorCode))
    print('motor halted')
    motor.disable()
    print('motor disabled')
