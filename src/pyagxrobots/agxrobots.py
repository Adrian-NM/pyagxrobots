#!/usr/bin/env python3
# coding=UTF-8

import sys
import os
import asyncio
import can
import time
import threading
import ctypes
import struct
import math
from can import Message
from typing import ByteString, Set, Type
from sqlite3.dbapi2 import Date

from . import UGVConfigMsg as UGVBaseMsg

# Type of current operating system
# posix： Linux
# nt: Windows
# java:

os_is_nt = os.name == 'nt'
# Linux and Mac are both part of the posix standard
# posix: Portable API for Unix-like operating systems
os_is_posix = os.name == 'posix'


class CanMsgsGet:
    def __init__(self, capacity=1024 * 4):
        self.size = 0
        self.rear = 0
        self.front = 0
        self.array = capacity

    def CanMsgsProcessV1(self, msg):
        if (msg.arbitration_id == UGVBaseMsg.CanIDV1.SYSTEM_STATE_ID):
            vehicle_state = int(msg.data[0])
            UGVBaseMsg.SetVehicleState(vehicle_state)
            control_mode = msg.data[1]
            UGVBaseMsg.SetControlMode(control_mode)
            battery_voltage = float((msg.data[2] & 0xff) << 8
                                    | msg.data[3]) / 10
            UGVBaseMsg.SetBatteryVoltage(battery_voltage)
            error_code = (msg.data[4] << 8) | msg.data[5]
            UGVBaseMsg.SetErrorCode(error_code)

            # print(
            #     'vehicle_state:%s control_mode:%s battery_voltage:%s error_code:%s  count_num:%s'
            #     % (vehicle_state, control_mode, battery_voltage, error_code,
            #        count_num))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV1.MOTION_STATE_ID):
            linear_velocity_get = ctypes.c_int16((msg.data[0] & 0xff) << 8
                                                 | msg.data[1])

            linear_velocity = float(linear_velocity_get.value / 1000)
            # print(int(linear_velocity))
            UGVBaseMsg.SetLinearVelocity(linear_velocity)
            angular_velocity_get = ctypes.c_int16((msg.data[2] & 0xff) << 8
                                                  | msg.data[3])
            angular_velocity = float(angular_velocity_get.value / 1000)
            UGVBaseMsg.SetAngularVelocity(angular_velocity)
            lateral_velocity_get = ctypes.c_int16((msg.data[4] & 0xff) << 8
                                                  | msg.data[5])
            lateral_velocity = float(lateral_velocity_get.value / 1000)
            UGVBaseMsg.SetLateralVelocity(lateral_velocity)

            # print(msg)

            # print(msg)
            # print(
            #     'linear_velocity:%s angular_velocity:%s lateral_velocity:%s steering_angle:%s '
            #     % (linear_velocity, angular_velocity, lateral_velocity,
            #        steering_angle))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV1.LIGHT_STATE_ID):
            light_cmd_ctrl = (msg.data[0])
            UGVBaseMsg.SetLightCmdCtrl(light_cmd_ctrl)
            front_mode = (msg.data[1])
            UGVBaseMsg.SetFrontMode(front_mode)
            front_custom = (msg.data[2])
            UGVBaseMsg.SetFrontCustom(front_custom)
            rear_mode = (msg.data[3])
            UGVBaseMsg.SetRearMode(rear_mode)
            rear_custom = (msg.data[4])
            UGVBaseMsg.SetRearCustom(rear_custom)

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV1.VALUE_SET_STATE_ID):
            if (msg.data[0] == 0xaa):
                base_neutral = True
            else:
                base_neutral = False
            UGVBaseMsg.SetBaseNeutral(base_neutral)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR1_STATE_ID):
            current1 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetCurrent1(current1)
            rpm1 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetRpm1(rpm1)
            driver1_temp = msg.date[4]
            UGVBaseMsg.SetDriver1State(driver1_temp)
            motor1_temp = msg.data[5]
            UGVBaseMsg.SetMotor1Temp(motor1_temp)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR2_STATE_ID):
            current2 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetCurrent2(current2)
            rpm2 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetRpm2(rpm2)
            driver2_temp = msg.date[4]
            UGVBaseMsg.SetDriver2State(driver2_temp)
            motor2_temp = msg.data[5]
            UGVBaseMsg.SetMotor2Temp(motor2_temp)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR3_STATE_ID):
            current3 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetCurrent3(current3)
            rpm3 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetRpm3(rpm3)
            driver3_temp = msg.date[4]
            UGVBaseMsg.SetDriver3State(driver3_temp)
            motor3_temp = msg.data[5]
            UGVBaseMsg.SetMotor3Temp(motor3_temp)

        elif(msg.arbitration_id == UGVBaseMsg.CanIDV1.ACTUATOR4_STATE_ID):
            current4 = float((msg.data[0] << 8 | msg.data[1])/10.0)
            UGVBaseMsg.SetCurrent4(current4)
            rpm4 = ctypes.c_int16(msg.data[2] << 8 | msg.data[3]).value
            UGVBaseMsg.SetRpm4(rpm4)
            driver4_temp = msg.date[4]
            UGVBaseMsg.SetDriver4State(driver4_temp)
            motor4_temp = msg.data[5]
            UGVBaseMsg.SetMotor4Temp(motor4_temp)

    def CanMsgsProcessV2(self, msg):
        """
        Process Can bus data according to ID
        """

        msg = msg
        if (msg.arbitration_id == UGVBaseMsg.CanIDV2.SYSTEM_STATE_ID):
            vehicle_state = int(msg.data[0])
            UGVBaseMsg.SetVehicleState(vehicle_state)
            control_mode = msg.data[1]
            UGVBaseMsg.SetControlMode(control_mode)
            battery_voltage = float((msg.data[2] & 0xff) << 8
                                    | msg.data[3]) / 10
            UGVBaseMsg.SetBatteryVoltage(battery_voltage)
            error_code = msg.data[5]
            UGVBaseMsg.SetErrorCode(error_code)
            count_num = msg.data[7]

            # print(
            #     'vehicle_state:%s control_mode:%s battery_voltage:%s error_code:%s  count_num:%s'
            #     % (vehicle_state, control_mode, battery_voltage, error_code,
            #        count_num))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.MOTION_STATE_ID):
            linear_velocity_get = ctypes.c_int16((msg.data[0] & 0xff) << 8
                                                 | msg.data[1])

            linear_velocity = float(linear_velocity_get.value / 1000)
            # print(int(linear_velocity))
            UGVBaseMsg.SetLinearVelocity(linear_velocity)
            angular_velocity_get = ctypes.c_int16((msg.data[2] & 0xff) << 8
                                                  | msg.data[3])
            angular_velocity = float(angular_velocity_get.value / 1000)
            UGVBaseMsg.SetAngularVelocity(angular_velocity)
            lateral_velocity_get = ctypes.c_int16((msg.data[4] & 0xff) << 8
                                                  | msg.data[5])
            lateral_velocity = float(lateral_velocity_get.value / 1000)
            UGVBaseMsg.SetLateralVelocity(lateral_velocity)
            steering_angle_get = ctypes.c_int16((msg.data[6] & 0xff) << 8
                                                | msg.data[7])
            steering_angle = float(steering_angle_get.value / 1000)
            UGVBaseMsg.SetSteeringAngle(steering_angle)
            # print(msg)

            # print(msg)
            # print(
            #     'linear_velocity:%s angular_velocity:%s lateral_velocity:%s steering_angle:%s '
            #     % (linear_velocity, angular_velocity, lateral_velocity,
            #        steering_angle))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR1_HS_STATE_ID):
            rpm1 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm1(rpm1)
            current1 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent1(current1)
            pulse_count1 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount1(pulse_count1)
            # print('rpm1:%s current1:%s pulse_count1:%s ' %
            #       (rpm1, current1, pulse_count1))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR2_HS_STATE_ID):
            rpm2 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm2(rpm2)
            current2 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent2(current2)
            pulse_count2 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount2(pulse_count2)
            # print('rpm2:%s current2:%s pulse_count2:%s ' %
            #       (rpm2, current2, pulse_count2))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR3_HS_STATE_ID):
            rpm3 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm3(rpm3)
            current3 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent3(current3)
            pulse_count3 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount3(pulse_count3)
            # print('rpm3:%s current3:%s pulse_count3:%s ' %
            #       (rpm3, current3, pulse_count3))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR4_HS_STATE_ID):
            rpm4 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm4(rpm4)
            current4 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent4(current4)
            pulse_count4 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount4(pulse_count4)

            # print('rpm4:%s current4:%s pulse_count4:%s ' %
            #       (rpm4, current4, pulse_count4))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR1_LS_STATE_ID):
            driver1_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver1Voltage(driver1_voltage)
            driver1_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver1Temp(driver1_temp)
            motor1_temp = msg.data[4]
            UGVBaseMsg.SetMotor1Temp(motor1_temp)
            driver1_state = msg.data[5]
            UGVBaseMsg.SetDriver1State(driver1_state)
            # print(
            #     'driver_voltage1:%s driver_temp1:%s motor_temp%s driver_state1:%s '
            #     % (driver1_voltage, driver1_temp, motor1_temp, driver1_state))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR2_LS_STATE_ID):
            driver2_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver2Voltage(driver2_voltage)
            driver2_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver2Temp(driver2_temp)
            motor2_temp = msg.data[4]
            UGVBaseMsg.SetMotor2Temp(motor2_temp)
            driver2_state = msg.data[5]
            UGVBaseMsg.SetDriver2State(driver2_state)
            # print(
            #     'driver_voltage2:%s driver_temp2:%s  motor_temp2:%s driver_state2:%s '
            #     % (driver2_voltage, driver2_temp, motor2_temp, driver2_state))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR3_LS_STATE_ID):
            driver3_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver3Voltage(driver3_voltage)
            driver3_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver3Temp(driver3_temp)
            motor3_temp = msg.data[4]
            UGVBaseMsg.SetMotor3Temp(motor3_temp)
            driver3_state = msg.data[5]
            UGVBaseMsg.SetDriver3State(driver3_state)
            # print(
            #     'driver_voltage3:%s driver_temp3:%s  motor_temp3:%s driver_state3:%s '
            #     % (driver3_voltage, driver3_temp, motor3_temp, driver3_state))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ACTUATOR4_LS_STATE_ID):
            driver4_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver4Voltage(driver4_voltage)
            driver4_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver4Temp(driver4_temp)
            motor4_temp = msg.data[4]
            UGVBaseMsg.SetMotor4Temp(motor4_temp)
            driver4_state = msg.data[5]
            UGVBaseMsg.SetDriver4State(driver4_state)
            # print(
            #     'driver_voltage4:%s driver_temp4:%s  motor_temp:%s driver_state4:%s '
            #     % (driver4_voltage, driver4_temp, motor4_temp, driver4_state))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.LIGHT_STATE_ID):
            light_cmd_ctrl = (msg.data[0])
            UGVBaseMsg.SetLightCmdCtrl(light_cmd_ctrl)
            front_mode = (msg.data[1])
            UGVBaseMsg.SetFrontMode(front_mode)
            front_custom = (msg.data[2])
            UGVBaseMsg.SetFrontCustom(front_custom)
            rear_mode = (msg.data[3])
            UGVBaseMsg.SetRearMode(rear_mode)
            rear_custom = (msg.data[4])
            UGVBaseMsg.SetRearCustom(rear_custom)

            # print(
            #     'enable_cmd_ctrl:%s front_mode:%s  front_custom:%s rear_mode:%s  rear_custom:%s'
            #     % (light_cmd_ctrl, front_mode, front_custom, rear_mode,
            #        rear_custom))
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.VERSION_RESPONSE_ID):
            control_hardware_version = int((msg.data[0] & 0xff) << 8
                                           | msg.data[1])
            UGVBaseMsg.SetControlHardwareVersion(control_hardware_version)
            actuaror_hardware_version = int((msg.data[2] & 0xff) << 8
                                            | msg.data[3])
            UGVBaseMsg.SetActuarorHardwareVersion(actuaror_hardware_version)
            control_software_version = int((msg.data[4] & 0xff) << 8
                                           | msg.data[5])
            UGVBaseMsg.SetControlSoftwareVersion(control_software_version)
            actuaror_software_version = int((msg.data[6] & 0xff) << 8
                                            | msg.data[7])
            UGVBaseMsg.SetActuarorSoftwareVersion(actuaror_software_version)
            # print(
            #     'control_hardware_version: %s actuaror_hardware_version: %s' %
            #     (control_hardware_version, actuaror_hardware_version))
            # print('control_software_version:%s actuaror_software_version:%s ' %
            #       (control_software_version, actuaror_software_version))

        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.ODOMETRY_ID):
            left_wheel_get = ctypes.c_int((msg.data[0] & 0xff) << 24
                                          | (msg.data[1] & 0xff) << 16
                                          | (msg.data[2] & 0xff) << 8
                                          | msg.data[3])
            left_wheel = left_wheel_get.value
            UGVBaseMsg.SetLeftWheel(left_wheel)
            right_wheel_get = ctypes.c_int((msg.data[4] & 0xff) << 24
                                           | (msg.data[5] & 0xff) << 16
                                           | (msg.data[6] & 0xff) << 8
                                           | msg.data[7])
            # print('left_wheel: %s right_wheel: %s ' %
            #       (left_wheel, right_wheel))
            right_wheel = right_wheel_get.value
            UGVBaseMsg.SetRightWheel(right_wheel)
            # print(msg)
        elif (msg.arbitration_id == UGVBaseMsg.CanIDV2.RC_STATE_ID):
            sws = msg.data[0]
            UGVBaseMsg.SetSws(sws)
            stick_right_v = msg.data[1]
            UGVBaseMsg.SetStickRightV(stick_right_v)
            stick_right_h = msg.data[2]
            UGVBaseMsg.SetStickRightH(stick_right_h)
            stick_left_v = msg.data[3]
            UGVBaseMsg.SetStickLeftV(stick_left_v)
            stick_left_h = msg.data[4]
            UGVBaseMsg.SetStickLeftH(stick_left_h)
            var_a = msg.data[5]
            UGVBaseMsg.SetVarA(var_a)


class DeviceCan:
    """
    重构后的DeviceCan类，解决了资源泄漏、竞态条件和NoneType错误。
    """
    def __init__(self, bustype=None, channel=None, bitrate=None):
        UGVBaseMsg._init()
        # 保持向后兼容性，同时设置合理默认值
        self.bustype = bustype if bustype is not None else "socketcan"
        self.channel = channel if channel is not None else "can0"
        self.bitrate = bitrate if bitrate is not None else 500000
        
        self.bus = None
        self.notifier = None
        self.base_verson = 0

        try:
            # 1. 创建并打开总线
            self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype, bitrate=self.bitrate)
            
            # 2. 获取底盘协议版本
            self.base_verson = self._get_base_version_safely()
            if self.base_verson == 0:
                raise can.CanError(f"错误：在CAN总线 {self.channel} 上超时，未能检测到AgileX UGV。请检查物理连接和底盘电源。")

            # 3. 启动后台监听线程
            # can.Notifier会自己管理一个后台线程，我们只需要在最后停止它
            self.notifier = can.Notifier(self.bus, [self.ListenerMessage])
            
        except Exception as e:
            print(f"初始化DeviceCan失败: {e}")
            self.shutdown() # 如果初始化过程中任何一步失败，确保资源被清理
            raise  # 重新抛出异常，让上层知道初始化失败

    def _get_base_version_safely(self):
        """
        健壮的版本获取函数。
        循环接收消息直到找到版本信息或超时。
        """
        print("正在检测UGV协议版本...")
        start_time = time.time()
        timeout = 3.0  # 设置3秒超时
        
        while time.time() - start_time < timeout:
            msg = self.bus.recv(0.1) # 每次等待100ms
            if msg is not None: # 关键：检查msg是否为None
                if msg.arbitration_id == UGVBaseMsg.CanIDV1.SYSTEM_STATE_ID:
                    print("检测到V1协议 (ID 0x151)")
                    return 1
                # 注意：根据官方代码，V2的RC_STATE_ID是0x241，这可能是一个周期性发送的稳定信号
                elif msg.arbitration_id == UGVBaseMsg.CanIDV2.RC_STATE_ID:
                    print("检测到V2协议 (ID 0x241)")
                    return 2
        
        return 0 # 返回0表示失败

    def ListenerMessage(self, msg):
        """消息监听回调函数，用于解析和存储车辆状态"""
        SlectMsg = CanMsgsGet()
        if self.base_verson == 1:
            SlectMsg.CanMsgsProcessV1(msg)
        elif self.base_verson == 2:
            SlectMsg.CanMsgsProcessV2(msg)

    def CanSend(self, msg):
        """高效地发送CAN消息"""
        if self.bus is None:
            # print("错误：CAN总线未打开，无法发送消息。") # 避免过多打印
            return
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"消息发送失败: {e}")

    def shutdown(self):
        """
        优雅地关闭CAN总线和后台线程，解决资源泄漏问题。
        """
        if self.notifier:
            self.notifier.stop()
            self.notifier = None
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            # print("DeviceCan已关闭。") # 可选的调试信息

    def __del__(self):
        """对象销毁时的安全保障"""
        self.shutdown()


class UGV:
    """
        EnableCANCtrl()
        SendVersionRequest()
        SendErrorClearByte()
        EnableLightCtrl()
        DisableLightCtrl()
        LightFrontMode()
        SendLinerVelocity()
        SendAngularVelocity()

        GetLightMode()        
        GetSysVersion()
        GetLeftWheelOdem()
        GetRightWheelOdem()
        GetLinerVelocity()
        GetAngularVelocity()
        GetErrorCode()

    """

    def __init__(self, *device_args, **device_kwargs):
        self.device = None
        can_device_init_fn = DeviceCan.__init__
        args_names = can_device_init_fn.__code__.co_varnames[:
                                                             can_device_init_fn
                                                             .__code__.
                                                             co_argcount]
        args_dict = dict(zip(args_names, device_args))
        args_dict.update(device_kwargs)

        self.device = DeviceCan(**args_dict)
        self.base_version = self.device.base_verson
    UGVBaseMsg._init()

    def shutdown(self):
        """将关闭指令传递给底层的CAN设备"""
        if hasattr(self, 'device') and self.device is not None:
            self.device.shutdown()

    def __del__(self):
        self.shutdown()

    def SendMsg(self, msg):
        self.device.CanSend(msg)

    def GetMsg(self):
        return self.device.CanGet()

    def SendVersionRequest(self):
        """
        Version Information Request
        """
        self.msg = Message(arbitration_id=UGVBaseMsg.CanIDV2.VERSION_REQUEST_ID,
                           data=[0x01],
                           is_extended_id=False)
        self.SendMsg(self.msg)

    def EnableCANCtrl(self):
        """
        控制底盘处于Can控制模式，需将遥控SWB拨到最上方
        Control the chassis in Can control mode, you need to dial the remote control SWB to the top
        """

        self.msg = Message(arbitration_id=UGVBaseMsg.CanIDV2.CTRL_MODE_CONFIG_ID,
                           data=[0x01],
                           is_extended_id=False)
        self.SendMsg(self.msg)

    def SetMotionCommand(self,
                         linear_vel: float = 0.0,
                         angular_vel: float = 0.0,
                         lateral_vel: float = 0.0,
                         steering_angle: float = 0.0):

        if self.base_version == 1:
            if abs(angular_vel)>abs(steering_angle):
                angular=angular_vel
            else: angular = steering_angle
            msg = Message(arbitration_id=UGVBaseMsg.CanIDV1.MOTION_COMMAND_ID,
                          data=[
                              0x01,
                              0,
                              (int(linear_vel )& 0xff),
                              (int(angular) & 0xff),
                              (int(lateral_vel) & 0xff),
                              0,
                              0,
                              0,
                          ],
                          is_extended_id=False)
            self.SendMsg(msg)

        elif self.base_version == 2:
            linear_cmd = linear_vel * 1000
            angular_cmd = angular_vel * 1000
            lateral_cmd = lateral_vel * 1000
            steering_cmd = steering_angle * 1000
            msg = Message(arbitration_id=UGVBaseMsg.CanIDV2.MOTION_COMMAND_ID,
                          data=[
                              int(linear_cmd) >> 8 & 0xff,
                              (int(linear_cmd) & 0x00ff),
                              int(angular_cmd) >> 8 & 0xff,
                              (int(angular_cmd) & 0x00ff),
                              int(lateral_cmd) >> 8 & 0xff,
                              (int(lateral_cmd) & 0x00ff),
                              int(steering_cmd) >> 8 & 0xff,
                              (int(steering_cmd) & 0x00ff)
                          ],
                          is_extended_id=False)
            self.SendMsg(msg)

    def SetLightCommand(self, front_mode: int = 0, front_custom_value: int = 0, rear_mode: int = 0, rear_custom_value: int = 0):
        if self.base_version==1:
            msg = Message(arbitration_id=UGVBaseMsg.CanIDV1.LIGHT_COMMAND_ID,
                        data=[
                            0x01,
                            front_mode,
                            front_custom_value,
                            rear_mode,
                            rear_custom_value,
                            0,
                            0,
                            0
                        ],
                        is_extended_id=False)
        elif self.base_version==2:
            msg = Message(arbitration_id=UGVBaseMsg.CanIDV2.LIGHT_COMMAND_ID,
                        data=[
                            0x01,
                            front_mode,
                            front_custom_value,
                            rear_mode,
                            rear_custom_value,
                            0,
                            0,
                            0
                        ],
                        is_extended_id=False)
        self.SendMsg(msg)
