#! /usr/bin/env python3

import rospy
import serial, time, os
from serial import SerialException
import numpy as np
from mdek_driver.msg import UWB
from geometry_msgs.msg import Point  # 태그 위치 메시지

class Ranges:
    def __init__(self):
        rospy.init_node('mdek_driver')
        
        # 기존 거리 데이터 토픽
        self.rangePub = rospy.Publisher("/Crawler/ranges", UWB, queue_size=10)
        
        # 태그 위치 데이터 토픽
        self.tagPosPub = rospy.Publisher("/Crawler/tag_position", Point, queue_size=10)

        dwPort = rospy.get_param('~port', '/dev/ttyACM0')
        dwRate = rospy.get_param('~baud', 115200)
        self.ser = serial.Serial(port=dwPort, timeout=10, baudrate=dwRate)

        self.Z = np.mat([[15.0], [15.0], [15.0], [15.0]])  # 태그-앵커 거리값 초기화

    def run(self):
        try:
            self.ser.close()
            self.ser.open()
            time.sleep(1)

            self.ser.write(b"nmt\r")
            time.sleep(1)
            self.ser.write(b"\r")
            self.ser.write(b"\r")
            time.sleep(1)

            self.ser.write(b"lec\r")
            time.sleep(0.1)

            msg = UWB()
            print("✅ Serial connection established.")

            while not rospy.is_shutdown():
                raw_data = self.ser.readline().decode().strip()
                print(f"🔹 Raw data: {raw_data}")

                if "DIST" in raw_data:
                    parts = raw_data.split(",")

                    try:
                        # 태그-앵커 거리값 추출
                        for i in range(4):  # AN0~AN3
                            anchor_index = parts.index(f"AN{i}")
                            self.Z[i] = float(parts[anchor_index + 5])  # 태그-앵커 거리값 저장

                        # 태그 위치값(POS) 추출
                        if "POS" in parts:
                            pos_index = parts.index("POS")
                            tag_x = float(parts[pos_index + 1])
                            tag_y = float(parts[pos_index + 2])
                            tag_z = float(parts[pos_index + 3])

                            # 태그 위치 메시지 생성 및 발행
                            tag_position = Point(x=tag_x, y=tag_y, z=tag_z)
                            self.tagPosPub.publish(tag_position)

                            rospy.loginfo(f"📡 Published Tag Position: x={tag_x}, y={tag_y}, z={tag_z}")

                    except (ValueError, IndexError) as e:
                        print(f"❌ Parsing Error: {parts}, {e}")

                print(f"🔹 Distances: {self.Z.T}")

                # 기존 거리 메시지 발행
                msg.rt1 = self.Z[0,0]
                msg.rt2 = self.Z[1,0]
                msg.rt3 = self.Z[2,0]
                msg.rt4 = self.Z[3,0]

                msg.header.frame_id = "UWB_frame"
                msg.header.stamp = rospy.get_rostime()
                self.rangePub.publish(msg)

            self.ser.close()

        except SerialException:
            print("❌ Serial port error")

if __name__ == "__main__":
    demo = Ranges()
    demo.run()

