import serial
import time
import math
import cv2  # OpenCV 사용
import numpy as np
import subprocess
import os
from calc_depth import calc_real_depth_and_save, calc_real_depth
from get_unique_name import get_unique_name

############### modify right-rotate angle

# 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM0', 9600)  # 포트 번호는 환경에 맞게 변경
time.sleep(2)  # 아두이노 초기화 시간 대기

base_dir = "depth_map"
unique_dir = get_unique_name(base_name=base_dir, is_directory=True)
os.makedirs(unique_dir, exist_ok=False)
print(f"make dir named {unique_dir}")

# 로봇의 전진 시 거리 (예: 2cm)
FORWARD = 2
MIN_DISTANCE = 6
MIN_BLOCK_DISTANCE = 5
WIDTH = 10
MAX_ANGLE = 35
MIN_ANGLE = 20
MAX_DISTANCE = 40
WIDTH_OF_FRAME = 316    # PIXY pixels -> (0 ~ 315)

# 이전 프레임 데이터 저장 변수
prev_depth_left = None
prev_depth_right = None

def turn_left(angle, forward):
    print(f"turn left {angle}\' & forward = {forward}")
    command = f"turn_left {angle} {forward}\n"
    arduino.write(command.encode())
    print("Waiting in left....")
    distance, x = wait_for_response()
    print("Wake up in left!")

    return distance, x

def turn_right(angle, forward):
    print(f"turn right {angle}\' & forward = {forward}")
    command = f"turn_right {angle} {forward}\n"
    arduino.write(command.encode())
    print("Waiting in right....")
    distance, x = wait_for_response()      
    print("Wake up in right!")

    return distance, x

# Move always 2cm 
def move_front(left_depth, right_depth):
    # Create a command string including left and right depth
    command = f"move_front {left_depth} {right_depth}\n"
    arduino.write(command.encode())  # Send the command to Arduino
    print("Waiting in front....")
    distance, x = wait_for_response()
    print("Wake up in front!")

    return distance, x

# Move front when meet corner
def move_FRONT():
    # Create a command string including left and right depth
    command = f"move_FRONT\n"
    arduino.write(command.encode())  # Send the command to Arduino
    print("Waiting in FRONT....")
    distance, x = wait_for_response()
    print("Wake up in FRONT!")

    return distance, x

# So close to Wall
def weenie(depth_LCR, delta, x):
    """
    depth_LCR = (left, center, right)
    delta = (left, right)
    """
    isWeenie = False

    print(f"check isWeenie / center = {depth_LCR[1]}")

    if depth_LCR[1] < MIN_DISTANCE:
        isWeenie = True
        if depth_LCR[0] > depth_LCR[2]:
            t = calculate_rotation_angle(delta[1], FORWARD)
            turn_left(15, 1)
        else:
            t = calculate_rotation_angle(delta[0], FORWARD)
            turn_right(15, 1)
    elif depth_LCR[0] < MIN_DISTANCE:
        isWeenie = True
        if depth_LCR[2] < MIN_DISTANCE:
            if depth_LCR[0] > depth_LCR[2]:
                t = calculate_rotation_angle(delta[1], FORWARD)
                turn_left(15, 1)
            else:
                t = calculate_rotation_angle(delta[0], FORWARD)
                turn_right(15, 1)
    elif depth_LCR[2] < MIN_DISTANCE:
        isWeenie = True
        t = calculate_rotation_angle(delta[1], FORWARD)
        turn_left(15, 1)

    depth_center = depth_LCR[1]
    if isWeenie:
        print("Weenie goes hut....")
        arduino.write(f"Null\n".encode())
        depth_center, x = wait_for_response()

    frame = get_frame()
    depth_left, depth_right, depth_block, _ = calc_real_depth(frame, depth_center, x)
    depth_LCR_rv = [depth_left, depth_center, depth_right]

    return isWeenie, depth_LCR_rv, depth_block, x

def wait_for_response():
    init = True
    while True:
        if init:
            print("Waiting for response....")
            init = False
        if arduino.in_waiting > 0:
            data = arduino.readline().decode().strip()
            print(f"Raw response: {data}")  # Debug raw data
            try:
                distance, x = data.split()
                distance = float(distance)
                x = int(x)
                if x < 0 or x > WIDTH_OF_FRAME:
                    print(f"Invalid block position x: {x}")
                    x = -1
                print(f"Distance: {distance}, x of block: {x}")
                return distance, x
            except ValueError:
                print(f"Malformed data received: {data}")

def get_frame():
    result = subprocess.run(['./get_raw_frame'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
        print("get_frame program failed:")
        print(result.stderr.decode())
        exit(1)

    frame = cv2.imread('out.ppm')

    if frame is None:
        print("Failed to read frame image")
        exit(1)
    return frame

def calculate_rotation_angle(delta, d):
    numerator = abs(delta) * math.sin(math.radians(40))
    denominator = d - delta * math.cos(math.radians(40))
    if denominator <= 0.01:
        # 분모가 0 이하인 경우 회전 각도를 최대 threshold로 제한
        t = MAX_ANGLE
    else:
        t = math.degrees(math.atan(numerator / denominator))
    
    if t > MAX_ANGLE:
        t = MAX_ANGLE
    elif t < MIN_ANGLE:
        t = MIN_ANGLE

    return int(t)

def calculate_angle_of_block(x, d):
    """
    x = x position of block
    d = distance from pixy to block
    """
    t = 0
    direction = None
    if abs(x - 158) < 2:
        t = 0
    elif x > 316:
        print("Block is out of range!")
        t = -1
    elif x < 158:
        delta = (158 - x) / WIDTH
        t = math.degrees(math.asin(delta / d))
        direction = 'left'
    elif x > 158:
        delta = (x - 158) / WIDTH
        t = math.degrees(math.asin(delta / d))
        direction = 'right'

    return t, direction

def main():
    global prev_depth_left, prev_depth_right
    step = 1

    print("Ready!")
    while True:
        arduino.write(f"step {step}\n".encode())
        print(f"step {step}")
        step += 1

        time.sleep(0.5)
        ultrasonic_distance, x = wait_for_response()
        frame = get_frame()

        # 프레임 좌우 끝과 중앙의 깊이 값 추출
        depth_center = ultrasonic_distance
        if depth_center > MAX_DISTANCE:
            depth_center = MAX_DISTANCE
        depth_left, depth_right, depth_block, _ = calc_real_depth_and_save(frame, depth_center, x, out_dir=unique_dir)
        print(f"{depth_left} -- {depth_center} -- {depth_right}")

        if depth_block < MIN_DISTANCE and depth_block != -1:
            print("Arrive at object!")
            break
        if depth_center + depth_left + depth_right < MIN_DISTANCE - 2:
            print("No way!")
            break

        depth_LCR = [depth_left, depth_center, depth_right]

        # 이전 프레임과의 깊이 차이 계산
        if prev_depth_left and prev_depth_right:
            delta_left = (depth_left - prev_depth_left)
            delta_right = (depth_right - prev_depth_right)
            # delta_left_rate = abs(delta_left) / prev_depth_left
            # delta_right_rate = abs(delta_right / prev_depth_right)

            delta = [delta_left, delta_right]

            isWeenie = False
            # so close to wall
            isWeenie, depth_LCR, depth_block, x = weenie(depth_LCR, delta, x)

            if depth_block != -1:
                if not isWeenie:
                    move_FRONT()
                    continue

            # both side is empty
            if depth_left > 50 and depth_right > 50:
                if not isWeenie:
                    move_FRONT()
            # 코너에 접근한 경우 처리
            elif depth_left > depth_center or depth_right > depth_center: 
                case = 1 if depth_left > depth_right else 2

                if depth_left > depth_center and case !=2:
                    print("meet to-left corner")
                    isWeenie, depth_LCR, depth_block, x = weenie(depth_LCR, delta, x)
                    for _ in range(2):
                        depth_LCR[1], x = move_FRONT()
                        isWeenie, depth_LCR, depth_block, x = weenie(depth_LCR, delta, x)
                        if isWeenie:
                            break
                        if depth_block < MIN_DISTANCE and depth_block != -1:
                            print("Arrive at object!")
                            break
                        frame = get_frame()
                        depth_LCR[0], depth_LCR[2], depth_block, _ = calc_real_depth(frame, depth_LCR[1], x)

                    t = calculate_rotation_angle(delta_left, FORWARD)
                    turn_left(t, 1)

                           
                elif depth_right > depth_center:
                    print("meet to-right corner")
                    isWeenie, depth_LCR, depth_block, x = weenie(depth_LCR, delta, x)
                    for _ in range(1):
                        depth_LCR[1], x = move_FRONT()
                        isWeenie, depth_LCR, depth_block, x = weenie(depth_LCR, delta, x)
                        if isWeenie:
                            break
                        if depth_block < MIN_DISTANCE and depth_block != -1:
                            print("Arrive at object!")
                            break
                        frame = get_frame()
                        depth_LCR[0], depth_LCR[2], depth_block, _ = calc_real_depth(frame, depth_LCR[1], x)

                    t = calculate_rotation_angle(delta_right, FORWARD)
                    turn_right(t, 1)
            
            print("all weenie dead")
            move_FRONT()



        else:
            move_front(depth_left, depth_right)

        # 이전 깊이 값 업데이트
        prev_depth_left = depth_left
        prev_depth_right = depth_right

main()
