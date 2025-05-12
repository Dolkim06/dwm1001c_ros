#! /usr/bin/env python3

import rospy
import serial, time, os
from serial import SerialException
from mdek_driver.msg import UWB
from geometry_msgs.msg import Point

class Ranges:
    def __init__(self):
        rospy.init_node('mdek_driver')

        # 거리 및 위치 토픽 발행
        self.rangePub = rospy.Publisher("/ranges", UWB, queue_size=10)
        self.tagPosPub = rospy.Publisher("/tag_position", Point, queue_size=10)

        # 사용자 정의 앵커 ID 순서 (문자열로)
        self.custom_anchor_id_order = ["4395", "5981", "0029", "5517"]

        dwPort = rospy.get_param('~port', '/dev/ttyACM0')
        dwRate = rospy.get_param('~baud', 115200)

        try:
            self.ser = serial.Serial(port=dwPort, timeout=10, baudrate=dwRate)
            rospy.loginfo(f"Serial port {dwPort} opened at {dwRate} baud.")
        except SerialException as e:
            rospy.logerr(f"Failed to open serial port {dwPort}: {e}")

    def run(self):
        try:
            self.ser.close()
            self.ser.open()
            time.sleep(1)

            # 초기 설정 명령
            self.ser.write(b"nmt\r")
            time.sleep(1)
            self.ser.write(b"\r\r")
            time.sleep(1)
            self.ser.write(b"lec\r")
            time.sleep(0.1)

            rospy.loginfo("Serial connection established. Receiving data...")

            while not rospy.is_shutdown():
                try:
                    raw_data = self.ser.readline().decode().strip()
                    rospy.loginfo(f"Raw data: {raw_data}")

                    if "DIST" in raw_data:
                        parts = raw_data.split(",")
                        distances_by_id = {}

                        try:
                            # "AN" 블록 탐색
                            for i in range(len(parts)):
                                if parts[i].startswith("AN"):
                                    try:
                                        anchor_id = parts[i + 1]
                                        distance = float(parts[i + 5])     # 거리값
                                        distances_by_id[anchor_id] = distance
                                        #rospy.loginfo(f"Anchor {anchor_id} → Distance: {distance}")
                                    except (IndexError, ValueError):
                                        rospy.logwarn(f"Invalid anchor data block: {parts[i:i+6]}")

                            # 사용자 정의 ID 순서대로 정렬
                            sorted_distances = [distances_by_id.get(k, 0.0) for k in self.custom_anchor_id_order]
                            rospy.loginfo(f"Sorted Distances by ID: {list(zip(self.custom_anchor_id_order, sorted_distances))}")

                            # 거리 메시지 생성 및 발행
                            msg = UWB()
                            msg.rt1 = sorted_distances[0] if len(sorted_distances) > 0 else 0.0
                            msg.rt2 = sorted_distances[1] if len(sorted_distances) > 1 else 0.0
                            msg.rt3 = sorted_distances[2] if len(sorted_distances) > 2 else 0.0
                            msg.rt4 = sorted_distances[3] if len(sorted_distances) > 3 else 0.0
                            msg.header.frame_id = "UWB_frame"
                            msg.header.stamp = rospy.get_rostime()
                            self.rangePub.publish(msg)

                            # 태그 위치값(POS) 처리
                            if "POS" in parts:
                                try:
                                    pos_index = parts.index("POS")
                                    tag_x = float(parts[pos_index + 1])
                                    tag_y = float(parts[pos_index + 2])
                                    tag_z = float(parts[pos_index + 3])
                                    tag_position = Point(x=tag_x, y=tag_y, z=tag_z)
                                    self.tagPosPub.publish(tag_position)
                                    rospy.loginfo(f"Tag Position: x={tag_x}, y={tag_y}, z={tag_z}")
                                    
                                except (IndexError, ValueError):
                                    rospy.logwarn("Invalid POS data block")
                                   
                            print("-"*70)

                        except Exception as e:
                            rospy.logwarn(f"Data parsing error: {e}")

                except serial.SerialException as e:
                    rospy.logerr(f"Serial communication error: {e}")
                    break

            self.ser.close()

        except SerialException as e:
            rospy.logerr(f"Serial port error: {e}")

if __name__ == "__main__":
    try:
        demo = Ranges()
        demo.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node interrupted.")
