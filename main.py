import pyrealsense2 as rs
import struct
import math
import RPi.GPIO as GPIO
from socket import *
from time import sleep
from pyquaternion import Quaternion
addr = ('127.0.0.1',1180)

SOF = 0xA5
data_length = 30
seq = 0x00
crc8 = 0x00

cmd_id = 0x0302

key1 = 0xff
key2 = 0xff

location_x = 0.0
location_y = 0.0
location_z = 0.0
pitch = 0.0
yaw = 0.0
roll = 0.0
end_float = float('inf')

crc16 = 0x00

if __name__ == '__main__':
    s = socket(AF_INET,SOCK_DGRAM)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(18,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(27,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(22,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(4,GPIO.OUT)
    led_state = False
    led_toggle = 5
    GPIO.output(4,led_state)
    while(True):
        while(True):
            try:
                if GPIO.input(27):
                    break
                else:
                    sleep(0.25)
                    led_state = True
                    led_toggle = 5
                    GPIO.output(4,led_state)
            except KeyboardInterrupt:
                exit()
        try:
            print("Key 17 pressed!")
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.pose)
            pipe.start(cfg)
        except:
            print("T265 offline!")
            continue
        while GPIO.input(27):
            try:
                try:
                    frames = pipe.wait_for_frames()
                except:
                    print("T265 disconnected!")
                    pipe.stop()
                    break
                pose = frames.get_pose_frame()
                if pose:
                    led_toggle = led_toggle - 1
                    if led_toggle < 0:
                        led_toggle = 5
                        if led_state:
                            led_state = False
                        else:
                            led_state = True
                    GPIO.output(4,led_state)
                    data = pose.get_pose_data()
                    w = data.rotation.w
                    x = data.rotation.x
                    y = data.rotation.y
                    z = data.rotation.z
                    q = Quaternion(w,x,y,z)
                    euler = q.yaw_pitch_roll
                    pitch = euler[1]
                    yaw = euler[0]
                    roll = euler[2]
                    print("pitch: ", pitch * 180 / math.pi, "roll: ", roll * 180 / math.pi, "yaw: ", yaw * 180 / math.pi)
                    location_x = 1000 * data.translation.z
                    location_y = 1000 * data.translation.x
                    location_z = 1000 * data.translation.y
                    print("x: ", location_x, "y: ", location_y, "z: ", location_z)
                    if GPIO.input(27):
                        key_1 = 0x00
                    else:
                        key_1 = 0xff
                    if not GPIO.input(22):
                        print("data stabilized")
                        key_2 = 0x00
                    else:
                        key_2 = 0xff
                    seq += 1
                    if seq > 255:
                        seq = 0
                    head = struct.pack("<BHB", SOF, data_length, seq)
                    head = head + struct.pack("<B",0x5F) #place_holder for crc8
                    buff = head + struct.pack("<HBBfffffff", cmd_id, key_1, key_2, location_x, location_y, location_z, pitch, yaw, roll, end_float)
                    buff = buff + struct.pack("<H", 0x5FFF) #place_holder for crc16
                    print(buff)
                    s.sendto(buff,addr)
                    sleep(0.04)
            except KeyboardInterrupt:
                GPIO.output(4,False)
                pipe.stop()
                exit()
        pipe.stop()
        GPIO.output(4,False)
