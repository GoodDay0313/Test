#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
import random

import cv2
import mediapipe as mp
import numpy as np
import queue
import threading


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot,topping,**kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self.order_msg='A'
        self.msg=queue.Queue()
        self.pre_num=0
        self.turn=0.0
        self.direction=0.0
        # self.min=queue.Queue()
        # self.max=queue.Queue()
        self.turn_min=100.0
        self.turn_max=-100.0
        self.arg=''
        self.order_msg_topping_pos=topping
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        #Camera
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, min_detection_confidence=0.5)
        self.previous_turn = None

        self.position_home = [179.2, -42.1, 7.4, 186.7, 41.5, -1.6]  # angle
        self.position_jig_A_grab = [-257.3, -138.3, 198, 68.3, 86.1, -47.0]  # linear
        self.position_jig_B_grab = [-152.3, -129.0, 198, 4.8, 89.0, -90.7]  # linear
        self.position_jig_C_grab = [-76.6, -144.6, 198, 5.7, 88.9, -50.1]  # linear
        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59]  # Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1]  # Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.0, -25.6, -88.5, 95.8]  # linear
        # topping before icecream
        self.position_topping_A = [-200.3, 162.8, 359.9, -31.7, 87.8, 96.1]  # Linear
        self.position_topping_B = [106.5, -39.7, 15.0, 158.7, 40.4, 16.9]  # Angle
        self.position_topping_C = [43.6, 137.9, 350.1, -92.8, 87.5, 5.3]  # Linear
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3]  # Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2]  # angle
        # topping after icecream
        self.position_topping_A_later = [-197.7, 159.7, 305.4, 102.6, 89.3, -129.7]  # Linear
        self.position_topping_B_later = [-47.7, 159.7, 305.4, 102.6, 89.3, -129.7]  # Linear
        self.position_topping_C_later = [56.2, 142.7, 316.8, 162.2, 88.4, -92.0]  # Linear
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2]  # Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6]  # Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1]  # Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7]  # Linear
    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False
#---------------------------------------camera-------------------------------------
    def calculate_direction(self, keypoints):
            nose_tip = keypoints[1]
            left_nose = keypoints[279]
            right_nose = keypoints[49]

            midpoint = np.array([(left_nose[0] + right_nose[0]) / 2,
                                (left_nose[1] + right_nose[1]) / 2,
                                (left_nose[2] + right_nose[2]) / 2])

            perpendicular_up = np.array([midpoint[0], midpoint[1] - 50, midpoint[2]])

            yaw = self.get_angle_between_lines(midpoint, nose_tip, perpendicular_up)
            turn = self.get_angle_between_lines(midpoint, right_nose, nose_tip)

            return yaw, turn

    def get_angle_between_lines(self, midpoint, point1, point2):
        vector1 = np.array(point1) - np.array(midpoint)
        vector2 = np.array(point2) - np.array(midpoint)

        dot_product = np.dot(vector1, vector2)
        magnitude1 = np.linalg.norm(vector1)
        magnitude2 = np.linalg.norm(vector2)

        cosine_theta = dot_product / (magnitude1 * magnitude2)
        angle_in_degrees = np.degrees(np.arccos(cosine_theta))

        return angle_in_degrees

    def draw_face_mesh(self, image, keypoints):
        for i, landmark in enumerate(keypoints):
            x = int(landmark.x * image.shape[1])
            y = int(landmark.y * image.shape[0])
            cv2.circle(image, (x, y), 1, (0, 255, 255), -1)  # Draw the landmark point
            cv2.putText(image, str(i), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

    def camera_run(self):
        cap = cv2.VideoCapture(0)  # Capture video from the webcam
        
        while cap.isOpened():
            ret, image = cap.read()
            if not ret:
                break

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(image_rgb)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    #self.draw_face_mesh(image, face_landmarks.landmark)
                    keypoints = [(landmark.x, landmark.y, landmark.z) for landmark in face_landmarks.landmark]

                    yaw, self.turn = self.calculate_direction(keypoints)
                    self.msg.put(self.turn)
                    
                    # Calculate the difference from the previous turn value
                    # if self.previous_turn is not None:
                    #     turn_difference = turn - self.previous_turn
                    #     cv2.putText(image, f'Turn Difference: {turn_difference:.2f}°', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

                    # Update the previous turn value
                    self.previous_turn =self. turn
                    # self.turn_min = min(self.turn_min, turn)
                    # self.turn_max = max(self.turn_max, turn)
                    # self.min.put(self.turn_min)
                    # self.max.put(self.turn_max)

                    cv2.putText(image, f'Yaw: {yaw:.2f}° Turn: {self.turn:.2f}°', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow('Face Mesh', image)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on 'q' key
                break
#---------------------------------------motion-------------------------------------
    def motion_home(self):

            code = self._arm.set_cgpio_analog(0, 0)
            if not self._check_code(code, 'set_cgpio_analog'):
                return
            code = self._arm.set_cgpio_analog(1, 0)
            if not self._check_code(code, 'set_cgpio_analog'):
                return

            # press_up
            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return

            # Joint Motion
            self._angle_speed = 80
            self._angle_acc = 200
            
            print('motion_home start')
            code = self._arm.set_servo_angle(angle=self.position_home, speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            print('motion_home finish')





    def game_start_position(self):

        code = self._arm.set_servo_angle(angle=[269.7, -2, 82.5, 93.5, -3.2,-16], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
                return

    def counting_number(self):

        code = self._arm.set_servo_angle(angle=[269.7, -2, 88.3, 96.2, -3.2,-17.7], speed=20,
                                             mvacc=self._angle_acc/2, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
                return
        code = self._arm.set_servo_angle(angle=[269.7, -2, 82.5, 93.5, -3.2,-16], speed=20,
                                             mvacc=self._angle_acc/2, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
                return

    def turn_left(self):
        time.sleep(1)
        self.pre_num=float(self.msg.get())
        print(self.pre_num)
        self.counting_number()
        time.sleep(0.8)
        self.counting_number()
        time.sleep(0.8)
        code = self._arm.set_servo_angle(angle=[269.7, -2, 82.4, 93.2, -43.2,-13.2], speed=100,
                                             mvacc=500, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
                return
        # self.direction=self.turn
        time.sleep(1)
        self.direction=self.turn
        code = self._arm.set_servo_angle(angle=[269.7, -2, 82.5, 93.5, -3.2,-16], speed=20,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
                return
        #self.arg=pre_num-aft_num
    def turn_right(self):
        time.sleep(1)
        self.pre_num=float(self.msg.get())
        print(self.pre_num)
        self.counting_number()
        time.sleep(0.8)
        self.counting_number()
        time.sleep(0.8)
        code = self._arm.set_servo_angle(angle=[269.7, -2, 82.4, 96.7, 36.8,-9.1], speed=100,
                                             mvacc=500, wait=False, radius=20.0)  
        if not self._check_code(code, 'set_servo_angle'):
                return    
        # self.direction=self.turn
        time.sleep(1)
        self.direction=self.turn   
        code = self._arm.set_servo_angle(angle=[269.7, -2, 82.5, 93.5, -3.2,-16], speed=20,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
                return
        
        # self.arg=pre_num-aft_num
        

    def play_game(self):
        # 0또는 1을 랜덤으로 선택
        choice = random.randint(0, 1)
        

        if choice == 0:
            self.arg='left'
            self.turn_left() # 왼쪽 선택
        else:
            self.arg='right'
            self.turn_right() # 오른쪽 선택


    

        

    def run(self):
        try:
            vision_thread = threading.Thread(target=self.camera_run)
            vision_thread.daemon = True  # 프로그램 종료 시 스레드 종료
            vision_thread.start()
            self.game_start_position()
            time.sleep(5)
            repeat_count = int(input("게임을 몇 번 실행 하겠습니까?")) #게임 반복 횟수 입력받기

            for i in range(repeat_count):
                self.turn_max=-100
                self.turn_min=100
                print(f"==={i+1}번쨰 게임 시작")
                
                arg=''
                self.play_game()
                if (self.direction-self.pre_num)>0:
                    arg='left'
                    print("Robot:", self.arg)
                    print('You:',arg)
                elif (self.direction-self.pre_num)<0:
                    arg='right'
                    print("Robot:", self.arg)
                
                    print("You:",arg)
                    
                else:
                    print("No game")
                # print(self.turn_min)
                # print(self.turn_max)
                print(self.direction)
                print(self.pre_num)
                
                if self.arg!=arg:
                    print("바보")
                else:
                    print("다시 할거임?")
                time.sleep(3)
            time.sleep(5)
            self.motion_home()
        #     print(self.arg)
        #     #print(self.min.get())
        #     print(self.turn_max)
        #     if self.arg!=arg:
        #         print("바보")
        #     else:
        #         print("다시 할거임?")
            print('game finish')
        except ValueError:
            print("유효한 숫자를 입력하세요.")
        except Exception as e:
            print(f"오류 발생:{e}")
            
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)




if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.184', baud_checkset=False)
    robot_main = RobotMain(arm,'C')
    robot_main.run()

