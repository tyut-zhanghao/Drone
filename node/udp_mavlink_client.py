#!/usr/bin/env python3
 # -*- coding: UTF-8 -*-
import time
import socket
import threading

from mavlink import *

class DeviceUdp(object):
    """
    """
    def __init__(self, local_addr, target_addr=None):
        """
        local_port: local port to receive data
        For example, target_addr = ('192.168.4.1', 18750)
        """
        self.dev         = None
        self.local_addr  = local_addr
        self.target_addr = target_addr

        self.dev = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dev.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

        self.dev.bind(self.local_addr)

    def write(self, buf):
        send_callback = self.dev.sendto(buf, self.target_addr)
    def read(self, size):
        buf, source_addr = self.dev.recvfrom(size)
        return buf
    def close(self):
        try:
            self.dev.close()
        except Exception as e:
            print(e)


class MAVAPI(object):
    """
    """
    def __init__(self, local_addr, target_addr,positionx,positiony):
        # 必要的成员变量
        self.sys_id = 254
        self.cmp_id = 1
        self.tgt_system_id    = 1
        self.tgt_component_id = 1

        self.x = positionx
        self.y = positiony

        self.receiveX = None
        self.receivey = None
        self.sign = False
        # 实例化UDP底层通信类
        self.dev = DeviceUdp(local_addr, target_addr)
        # 实例化MAVLink通信类
        self.mav = MAVLink(self.dev, self.sys_id, self.cmp_id)

        # 用于保存接收到的mavlink消息
        self.cur_mav_msg = None
        # 系统启动时间
        self.sys_time_startup = time.time()
        # 各类线程实例，并开启线程
        # self.th_msg_receive = threading.Thread(daemon=True, target=self.th_receive_fun)
        # self.th_send_heartbeat = threading.Thread(daemon=True, target=self.th_send_heartbeat_fun)
        # self.th_msg_receive.start()
        # self.th_send_heartbeat.start()

    def th_receive_fun(self):

        # while True:
            data = self.dev.read(256)
            if len(data)>0:
                # 收到数据，解析mavlink消息
                try:
                    self.cur_mav_msg = self.mav.parse_char(data)
                except Exception as e:
                    pass
                    # continue
                if isinstance(self.cur_mav_msg, MAVLink_message):
                    pass

                # if isinstance(self.cur_mav_msg, MAVLink_heartbeat_message):
                #     print(self.cur_mav_msg)
                #     print("INFO: sys={}, cmp={}".format(self.cur_mav_msg.get_srcSystem(), self.cur_mav_msg.get_srcComponent()))
                #     print("INFO: type={}, autopilot={}".format(self.cur_mav_msg.type, self.cur_mav_msg.autopilot))

                if isinstance(self.cur_mav_msg, MAVLink_local_position_ned_message):
                    # print("INFO: roll={:.3f}, pitch={:.3f}, yaw={:.3f}".format(self.cur_mav_msg.roll, self.cur_mav_msg.pitch, self.cur_mav_msg.yaw))
                    self.receiveX  = self.cur_mav_msg.x
                    self.receiveY = self.cur_mav_msg.y
                    self.sign = True
                    # self.yaw_NED = self.cur_mav_msg.yaw
                   
    def th_send_heartbeat_fun(self):
        """
        线程回调函数：以一定频率发送心跳包
        """
        for i in range(10):
            ### send as a onboard computer or GCS
            # msg = MAVLink_heartbeat_message(type=MAV_TYPE_GCS, autopilot=MAV_AUTOPILOT_INVALID, base_mode=0, custom_mode=0, 
            #                                system_status=MAV_STATE_ACTIVE, mavlink_version=3)
            ### send as a drone simulator
            msg = MAVLink_local_position_ned_message(time_boot_ms=0.1,x=self.x,y=self.y,z=0,vx=0,vy=0,vz=0)
            # msg = MAVLink_local_position_ned_message(type=MAV_TYPE_QUADROTOR, autopilot=MAV_AUTOPILOT_PX4, base_mode=MAV_MODE_PREFLIGHT, custom_mode=0x60000, 
                                            # system_status=MAV_STATE_ACTIVE, mavlink_version=3)
            self.mav.send(msg)
            print('send')
            time.sleep(0.1)

    def arm_disarm(self, arm):
        """
        发送解锁/上锁指令
        arm:
            - False: disarm
            - True: arm
        """
        arm = 1 if arm else 0
        msg_cmd = MAVLink_command_long_message(self.tgt_system_id, self.tgt_component_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0)
        self.mav.send(msg_cmd)
    
    def takeoff_land(self, cmd='takeoff'):
        """
        发送起飞指令，通过MAVLink SET_MODE(#11)消息实现原地起飞
        """
        base_mode   = 129
        if cmd == 'takeoff':
            custom_mode = 0x2040000
        elif cmd == 'land':
            custom_mode = 0x6040000
        
        msg = MAVLink_set_mode_message(self.tgt_system_id, base_mode=base_mode, custom_mode=custom_mode)
        self.mav.send(msg)

    def exit(self):
        """
        退出，关闭UDP通信
        """
        self.dev.close()

# def test_gcs():
#     local_addr  = ('', 14540)
#     target_addr = ('127.0.0.1', 14550)
#     mavapi = MAVAPI(local_addr, target_addr)
#     mavapi.exit()

def test_px4_sim():
    local_addr  = ('', 14540)
    target_addr = ('127.0.0.1', 14580)
    mavapi = MAVAPI(local_addr, target_addr)

    time.sleep(3)
    mavapi.arm_disarm(True) 
    time.sleep(2)
    mavapi.takeoff_land('takeoff')
    time.sleep(5)
    mavapi.takeoff_land('land')

def broadcastPose(positionx,positiony):
    local_addr = ('',9999)
    target_addr = ('<broadcast>',9999)
    x = positionx
    y = positiony
    mavapi = MAVAPI(local_addr,target_addr,x,y)
    mavapi.th_send_heartbeat_fun()
    
def recivePose():
    local_addr = ('',9999)
    target_addr = ('<broadcast>',9999)
    mavapi = MAVAPI(local_addr,target_addr)
    mavapi.th_receive_fun()
    mavapi.receiveX
    mavapi.receiveY

# if __name__ == '__main__':
#     # test_gcs()
#     test_px4_sim()
