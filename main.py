import pyrealsense2 as rs
import struct
import math
import RPi.GPIO as GPIO
from socket import *
from time import sleep
from pyquaternion import Quaternion
addr = ('127.0.0.1',1180)

pump_button = 22
enable_button = 27
z_enable_button = 17
reset_button = 18


class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, value):
        if self.value is None:
            self.value = value
        else:
            self.value = self.alpha * value + (1 - self.alpha) * self.value
        return self.value

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
    GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(18,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(27,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(22,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(4,GPIO.OUT)
    location_x_filter = LowPassFilter(alpha=0.2)
    location_y_filter = LowPassFilter(alpha=0.2)
    location_z_filter = LowPassFilter(alpha=0.2)
    pitch_filter = LowPassFilter(alpha=0.2)
    yaw_filter = LowPassFilter(alpha=0.2)
    roll_filter = LowPassFilter(alpha=0.2)

    led_state = False
    led_toggle = 5
    GPIO.output(4,led_state)
    reset_state = GPIO.input(reset_button)
    while(True):
        while(True):
            try:
                if GPIO.input(reset_button) != reset_state:
                    reset_state = GPIO.input(reset_button)
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
        while True:
            if GPIO.input(reset_button) != reset_state:
                reset_state = GPIO.input(reset_button)
                break
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
                    pitch = -euler[1] + math.pi / 2
                    yaw = euler[0]
                    roll = euler[2] - 1.57
                    print("pitch: ", pitch * 180 / math.pi, "roll: ", roll * 180 / math.pi, "yaw: ", yaw * 180 / math.pi)
                    location_x = -1800 * data.translation.z
                    location_y = -1800 * data.translation.x
                    location_z = 1000 * data.translation.y
                    print("x: ", location_x, "y: ", location_y, "z: ", location_z)
                    data_valid = location_z > 0 and location_z < 485 and location_y > 0  and pitch > 0 and pitch < math.pi and yaw > -math.pi and yaw < math.pi and math.sqrt(x**2 + y**2)< 460
                    if  GPIO.input(pump_button):
                        key_1 = 0x01
                    else:
                        key_1 = 0x00
                    if  GPIO.input(z_enable_button):
                        key_2 = 0x02
                    else:
                        pass
                    if not GPIO.input(enable_button) and data_valid:
                        print("data stabilized")
                        key_2 = 0x01
                    else:
                        key_2 = 0x00
                    seq += 1
                    if seq > 255:
                        seq = 0
                    pitch = pitch_filter.update(pitch)
                    yaw = yaw_filter.update(yaw)
                    roll = roll_filter.update(roll)
                    location_x = location_x_filter.update(location_x)
                    location_y = location_y_filter.update(location_y)
                    location_z = location_z_filter.update(location_z)
                    head = struct.pack("<BHB", SOF, data_length, seq)
                    head = head + struct.pack("<B",0x5F) #place_holder for crc8
                    buff = head + struct.pack("<HBBfffffff", cmd_id, key_2, key_1, location_x, location_y, location_z, pitch, roll, yaw, end_float)
                    buff = buff + struct.pack("<H", 0x5FFF) #place_holder for crc16
                    #l = [hex(int(i)) for i in buff]
                    #print(" ".join(l))
                    if data_valid:
                        s.sendto(buff,addr)
                    sleep(0.04)
            except KeyboardInterrupt:
                GPIO.output(4,False)
                pipe.stop()
                exit()
        pipe.stop()
        GPIO.output(4,False)
