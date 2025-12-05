import rclpy
import time
import DR_init
from std_msgs.msg import Int32, Bool

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VEL = 60
ACC = 60

current_progress = 0
stop_requested = False
recovery_requested = False
end_received = False
progress_pub = None  

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ============================================================
# ⭐ 전역 publish_state 함수 (리커버리에서도 사용 가능)
# ============================================================
def publish_state(num):
    global progress_pub, current_progress
    if progress_pub is None:
        print("⚠ progress_pub 초기화 필요")
        return

    msg = Int32()
    msg.data = num
    progress_pub.publish(msg)
    print(f"[GLOBAL] Progress Published → {num}")

    if num != 7:
        current_progress = num


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VEL: {VEL}")
    print(f"ACC: {ACC}")
    print("#" * 50)


# ============================================================
# 🔥 Recovery Motion
# ============================================================
def open_gripper():
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(1.0)


def close_gripper():
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(1, 1)
    set_digital_output(2, 1)
    time.sleep(1.0)

def water_close_gripper():
    from DSR_ROBOT2 import set_digital_output
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(1.0)



def home_return():
    from DSR_ROBOT2 import movej
    movej([0, 0, 90, 0, 90, 0], vel=VEL, acc=ACC)

def end_motion():
    from DSR_ROBOT2 import movel, movej, posx, DR_BASE
    move1 = [2.68, -32.42, 124.53, 35.66, 59.13, 0.00]
    move2 = [6.10, 23.80, 70.12, 31.39, 109.08, 0.00]
    movej(move1, 60, 60)
    close_gripper()
    open_gripper()
    movej(move2, 60, 60)
    close_gripper()
    open_gripper()

def recovery_motion():
    from DSR_ROBOT2 import movel, movej, posx, DR_BASE

    print("🔧 Recovery Motion 실행", current_progress)

    open_gripper()

    if current_progress == 1 or current_progress == 8 or current_progress == 2:
        L_pot_release_up = posx([340.00, 275.58, 454.66, 48.22, 177.91, -131.07])
        L_pot_grip = posx([408.41, -222.64, 481.84, 58.44, -179.90, -32.30])

        movel(L_pot_release_up, vel=VEL, acc=ACC)
        movel(posx([0, 0, -101, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        close_gripper()

        movel(posx([0, 0, 100, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        movel(L_pot_grip, vel=VEL, acc=ACC)
        movel(posx([0, 0, -182.6, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)

        open_gripper()
        print("냄비를 제자리에 놔뒀습니다.")
        home_return()

    else:
        L_pot_grip = posx([340.00, 275.58, 454.66, 48.25, 177.91, -39.92])
        J_pot_move = [-7.28, 16.79, 80.92, -1.40, 82.37, 83.53]

        movel(L_pot_grip, vel=VEL, acc=ACC)
        movej([0, 0, 0, 0, 0, -90], vel=VEL, acc=ACC, mod=1)

        movel(posx([0, 0, -101, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        close_gripper()
        movel(posx([0, 0, 80, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)

        home_return()
        movej([0, 0, 0, 0, 0, 90], vel=VEL, acc=ACC, mod=1)
        movej(J_pot_move, vel=VEL, acc=ACC)

        movel(posx([134, 0, 0, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        movel(posx([0, 0, -50, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)

        open_gripper()
        print("냄비를 버렸습니다.")
        home_return()

    print("🟢 Recovery Motion 완료!")
    

# ============================================================
# Perform Task
# ============================================================
def perform_task(node, mode):
    from DSR_ROBOT2 import movej, movel, posx, DR_BASE
    from DSR_ROBOT2 import set_digital_output, get_digital_input  # [MOD] DI 함수 import 추가
    global stop_requested

    progress_pub = node.create_publisher(Int32, "/progress_state", 10)

    def publish_state(num):
        msg = Int32()
        msg.data = num
        progress_pub.publish(msg)
        print(f"Progress Published → {num}")
        if num != 7:
            global current_progress
            current_progress = num
            
    def check_end():
        if end_received:
            print("🍜 END SIGNAL 감지 → 즉시 종료")
            movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
            raise Exception("END")
            
    def check_stop():
        if stop_requested:
            print("⛔ STOP 감지 — 즉시 중단!")
            movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
            raise Exception("STOP")

    def open_gripper():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        time.sleep(1.0)

    def water_close_gripper():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        time.sleep(1.0)


    def close_gripper():
        set_digital_output(1, 1)
        set_digital_output(2, 1)
        time.sleep(1.0)

    def home_return():
        movej([0, 0, 90, 0, 90, 0], vel=VEL, acc=ACC)

    # [MOD] 재료 소진 알림 및 그리퍼 센서 체크 함수 추가
    def notify_material_empty(step_name: str):
        """재료 소진 시 UI에 알리기 위한 퍼블리시 + 예외 발생"""
        msg = Bool()
        msg.data = True
        publish_state(7)
        print(f"❌ 재료 소진 감지 ({step_name}) → UI에 알림 전송")
        raise Exception("NO_MATERIAL")

    def check_grip(step_name: str):
        """그리퍼 DI(1) 상태를 읽어 재료 잡힘 여부 확인"""
        di = get_digital_input(1)
        print(f"[{step_name}] 그리퍼 DI 상태 = {di}")
        if di == 0:
            # 재료를 못 잡은 경우 → 재료 소진 처리
            notify_material_empty(step_name)
        print("================================",di)

    # === 조리 시퀀스 ===
    def move_pot():
        publish_state(1)
        check_stop()
        J_pot_grip = [-30.86, 16.01, 91.07, -0.22, 72.95, -30.77]
        L_pot_release_up = posx([340.00, 275.58, 455.66, 48.25, 177.91, -39.92])
        
        # 초기화
        home_return()
        open_gripper()
        
        # 냄비 집기
        movej(J_pot_grip, vel=VEL, acc=ACC)
        close_gripper()
        check_grip("냄비 집기")  # [MOD] 냄비 잡힘 여부 확인
        time.sleep(0.5)
        
        # 냄비 놓는 위치 상단으로 이동
        movel(L_pot_release_up, vel=VEL, acc=ACC)
        
        # 냄비 놓는 위치 하단으로 이동
        movel(posx([0, 0, -101, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        time.sleep(0.5)
        open_gripper()
        print("냄비를 놓았습니다.")
        
        # 집게 들기
        movel(posx([0, 0, 100, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        home_return()

    def drop_water():
        check_stop()
        publish_state(2)
        L_water_grip = posx([486.43, 134.10, 318.03, 10.36, 145.90, -83.99])
        L_water_out = posx([396.91, 118.61, 450.07, 9.98, 145.85, -84.27])
        L_water_drop = posx([134.57, 136.44, 430.30, 24.24, 158.55, -94.97])
        L_water_dropping_first = posx([208.78, 164.55, 412.99, 152.18, -173.66, 37.15])
        L_water_dropping_second = posx([281.85, 238.05, 411.82, 53.92, -151.42, -62.09])
        L_water_dropping_third = posx([350.03, 308.92, 335.97, 50.91, -120.55, -63.43])
        L_water_waste = posx([633.45, -67.80, 331.46, 11.74, 178.92, -79.46])

        water_VELACC = 10
        
        # 물병 디스펜서에서 잡기
        open_gripper()
        movel(L_water_out, vel=VEL, acc=ACC)
        movel(L_water_grip, vel=VEL, acc=ACC)
        time.sleep(1.0)
        water_close_gripper()
        time.sleep(0.5)
        check_grip("물병 집기")  # [MOD] 물병 잡힘 여부 확인
        
        # 물병 꺼내기
        movel(L_water_out, vel=VEL, acc=ACC)
        
        # 물병 따르는 위치로 이동
        movel(L_water_drop, vel=VEL, acc=ACC)
        
        # 물병 기울여서 따르기
        movel(L_water_dropping_first, vel=water_VELACC, acc=water_VELACC)
        time.sleep(2)
        print("1번완료")
        movel(L_water_dropping_second, vel=water_VELACC, acc=water_VELACC)
        time.sleep(2)
        print("2번완료")
        movel(L_water_dropping_third, vel=water_VELACC, acc=water_VELACC)
        time.sleep(2)
        print("물을 다 따랐습니다.")
        home_return()
        
        # 물병 버리기
        movel(L_water_waste, vel=VEL, acc=ACC)
        movel(posx([0, 0, -50, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        open_gripper()
        print("물병을 버렸습니다.")
        home_return()

    def take_noodle():
        publish_state(3)
        check_stop()
        L_noodle_case = posx([496.50, 286.33, 319.35, 5.35, 153.96, -90.67])
        L_noodle_out = posx([427.51, 281.88, 460.77, 4.86, 153.91, -91.04])
        L_noodle_upper_pot = posx([336.72, 57.84, 522.26, 74.88, 158.15, 159.68])
        L_noodle_down_pot = posx([342.26, 119.16, 360.56, 75.13, 158.31, 159.81])
        # 면 디스펜서에서 집기
        movel(L_noodle_case, vel=VEL, acc=ACC)
        close_gripper()
        check_grip("면 집기")
        time.sleep(0.5)
        # 면 빼기
        movel(L_noodle_out, vel=VEL, acc=ACC)
        # 면 넣기
        movel(L_noodle_upper_pot, vel=VEL, acc=ACC)
        time.sleep(0.3)
        movel(L_noodle_down_pot, vel=VEL, acc=ACC)
        open_gripper()
        print("면을 넣었습니다.")
        movel(posx([0, 0, 50, 0, 0, 0]), vel=VEL, acc=ACC, ref=DR_BASE, mod=1)
        home_return()

    def dance(n):
        
        # 초기 위치 및 목표 위치 설정
        J1 = [0.00, 16.11, 81.19, 12.54, -61.66, 0.00]
        J2 = [0.00, -16.87, 107.71, 12.77, -125.63, 0.00]
        open_gripper()
        # 반복 동작 수행
        for i in range(n):      
            check_stop()
            movej(J1, vel=60, acc=60)
            movej(J2, vel=60, acc=60)

    def pour_sauce():
        publish_state(4)
        check_stop()
        J_cup1_grip = [-1.91, 9.67, 116.94, -14.32, -29.80, -73.38]
        L_cup1_case = posx([613.63, 18.73, 285.86, 5.94, 97.63, -84.76])
        L_cup1_back = posx([382.21, 7.73, 303.07, 5.58, 97.51, -84.87])
        J_sauce_upper_pot = [39.23, -36.98, 125.59, -10.72, 49.46, -83.16]
        J_sauce_pour = [43.79, 26.84, 73.89, -21.61, 114.97, -111.39]
        J_cup2_grip = [-17.15, 8.15, 118.16, -37.14, -34.86, -56.71]
        L_cup2_case = posx([605.37, -83.77, 292.60, 4.50, 97.19, -85.68])
        L_cup2_back = posx([359.70, -89.65, 307.57, 3.47, 96.96, -86.01])
        J_cup3_grip = [-39.15, 4.52, 121.18, -62.17, -47.37, -40.28]
        L_cup3_case = posx([601.76, -190.73, 292.83, 3.14, 96.89, -86.31])
        L_cup3_back = posx([340.48, -198.37, 314.60, 2.14, 96.65, -86.58])

        VELACC_01 = 40
        VELACC_02 = 30

        if mode == 0:
            #--- 컵1 ---
            # 컵1 잡기
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵1 집기 (mode 0)")  # [MOD]
            movel(L_cup1_back, vel=VEL, acc=ACC)
            # 컵1 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵1 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup1_back, vel=VEL, acc=ACC)
            home_return()

        elif mode == 1:
            #--- 컵1 ---
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵1 집기 (mode 1)")  # [MOD]
            movel(L_cup1_back, vel=VEL, acc=ACC)
            # 컵1 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵1 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup1_back, vel=VEL, acc=ACC)
            home_return()

            #--- 컵2 ---
            movej(J_cup2_grip, vel=VEL, acc=ACC)
            movel(L_cup2_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵2 집기 (mode 1)")  # [MOD]
            movel(L_cup2_back, vel=VEL, acc=ACC)
            # 컵2 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵2 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup2_grip, vel=VEL, acc=ACC)
            movel(L_cup2_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup2_back, vel=VEL, acc=ACC)
            home_return()

        elif mode == 2:
            #--- 컵1 ---
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵1 집기 (mode 2)")  # [MOD]
            movel(L_cup1_back, vel=VEL, acc=ACC)
            # 컵1 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵1 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup1_back, vel=VEL, acc=ACC)
            home_return()

            #--- 컵3 ---
            movej(J_cup3_grip, vel=VEL, acc=ACC)
            movel(L_cup3_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵3 집기 (mode 2)")  # [MOD]
            movel(L_cup3_back, vel=VEL, acc=ACC)
            # 컵3 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵3 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup3_grip, vel=VEL, acc=ACC)
            movel(L_cup3_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup3_back, vel=VEL, acc=ACC)
            home_return()

        elif mode == 3:
            #--- 컵1 ---
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵1 집기 (mode 3)")  # [MOD]
            movel(L_cup1_back, vel=VEL, acc=ACC)
            # 컵1 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵1 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup1_grip, vel=VEL, acc=ACC)
            movel(L_cup1_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup1_back, vel=VEL, acc=ACC)
            home_return()

            #--- 컵2 ---
            movej(J_cup2_grip, vel=VEL, acc=ACC)
            movel(L_cup2_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵2 집기 (mode 3)")  # [MOD]
            movel(L_cup2_back, vel=VEL, acc=ACC)
            # 컵2 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵2 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup2_grip, vel=VEL, acc=ACC)
            movel(L_cup2_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup2_back, vel=VEL, acc=ACC)
            home_return()

            #--- 컵3 ---
            movej(J_cup3_grip, vel=VEL, acc=ACC)
            movel(L_cup3_case, vel=VEL, acc=ACC)
            close_gripper()
            check_grip("소스 컵3 집기 (mode 3)")  # [MOD]
            movel(L_cup3_back, vel=VEL, acc=ACC)
            # 컵3 붓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_sauce_pour, vel=VELACC_02, acc=VELACC_02)
            # 컵3 돌려놓기
            movej(J_sauce_upper_pot, vel=VELACC_01, acc=VELACC_01)
            time.sleep(0.5)
            movej(J_cup3_grip, vel=VEL, acc=ACC)
            movel(L_cup3_case, vel=VEL, acc=ACC)
            open_gripper()
            movel(L_cup3_back, vel=VEL, acc=ACC)
            home_return()
            
            
    
    # === 전체 조리 실행 ===
    # move_pot()
    # drop_water()

    # publish_state(6)  # 물 끓이는 중
    # dance(7)
    # home_return()
    # pour_sauce()
    home_return()
    open_gripper()
    take_noodle()

    # publish_state(5)  # 라면 끓이는 중
    # # ⭐ 최종 END SIGNAL 대기
    # print("🍜 조리 완료 → END SIGNAL 대기중…")
    # while not end_received:
    #     rclpy.spin_once(node, timeout_sec=0.2)
    #     dance(1)
    # publish_state(8)
    # end_motion()
    # publish_state(0)
    # home_return()

    

# ============================================================
# MAIN LOOP
# ============================================================
def main(args=None):
    global stop_requested, recovery_requested, progress_pub

    rclpy.init()
    node = rclpy.create_node("pot_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # ⭐⭐⭐ Progress Publisher 초기화 ⭐⭐⭐
    progress_pub = node.create_publisher(Int32, "/progress_state", 10)

    start_received = False
    mode = None

    # 콜백 등록
    def start_cb(msg):
        nonlocal start_received
        if msg.data:
            print("▶ START 수신")
            start_received = True

    def mode_cb(msg):
        nonlocal mode
        mode = msg.data
        print(f"▶ MODE 선택 = {mode}")

    def stop_cb(msg):
        global stop_requested
        if msg.data:
            print("⛔ STOP 수신")
            stop_requested = True

    def recovery_cb(msg):
        global recovery_requested
        if msg.data:
            print("🟢 RECOVERY 수신")
            recovery_requested = True
            
    def end_cb(msg):
        global end_received
        if msg.data:
            print("🍜 END SIGNAL 수신")
            end_received = True
            
    node.create_subscription(Bool, "/start_signal", start_cb, 10)
    node.create_subscription(Int32, "/mode_select", mode_cb, 10)
    node.create_subscription(Bool, "/stop_signal", stop_cb, 10)
    node.create_subscription(Bool, "/recovery_signal", recovery_cb, 10)
    node.create_subscription(Bool, "/end_signal", end_cb, 10)


    print("=== Robot Ready. Waiting... ===")

    # ======================
    # 메인 로직 시작
    # ======================
    while rclpy.ok():

        start_received = False
        mode = None
        stop_requested = False
        recovery_requested = False
        end_received = False  # ⭐ 매 사이클 초기화

        print("📌 START 신호 대기...")
        while rclpy.ok() and not start_received:
            rclpy.spin_once(node, timeout_sec=0.2)
            if stop_requested:
                break

        # STOP 먼저 들어온 경우
        if stop_requested:
            from DSR_ROBOT2 import movej
            print("⛔ STOP → 초기자세 복귀")

            try:
                movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
            except:
                pass

            print("🟢 Recovery 대기...")
            while not recovery_requested:
                rclpy.spin_once(node, timeout_sec=0.2)

            print("🟢 Recovery 수신 → Recovery Motion 실행")
            recovery_motion()

            # ⭐⭐⭐ 여기서 state = 0 발사 ⭐⭐⭐
            publish_state(0)

            stop_requested = False
            recovery_requested = False

            print("🔄 Recovery 완료 → START 재대기")
            continue

        print("📌 Mode 선택 대기...")
        while mode is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            if stop_requested:
                break

        if stop_requested:
            continue

        print("🍜 작업 시작!")
        try:
            initialize_robot()
            perform_task(node, mode)
            print("🎉 라면 완성!")

        except Exception as e:
            from DSR_ROBOT2 import movej

            # STOP 처리
            if str(e) == "STOP":
                print("⚠ 작업 중 STOP → 초기 자세 복귀")
                try:
                    movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
                except:
                    pass

                print("🟢 Recovery 대기...")
                while not recovery_requested:
                    rclpy.spin_once(node, timeout_sec=0.2)

                recovery_motion()
                publish_state(0)   # ⭐ 추가

                stop_requested = False
                recovery_requested = False
                print("🔄 Recovery 완료 → 처음부터 재시작")
                continue
                
            elif str(e) == "END":
                print("🍜 END 예외 처리 → 복구 실행")
                movej([0,0,90,0,90,0], vel=50, acc=50)
                recovery_motion()
                publish_state(0)
                continue
                
            # 재료 소진 처리
            elif str(e) == "NO_MATERIAL":
                print("⚠ 재료 소진 → 작업 중단")

                try:
                    movej([0, 0, 90, 0, 90, 0], vel=50, acc=50)
                except:
                    pass

                print("🟢 Recovery 대기...")
                while not recovery_requested:
                    rclpy.spin_once(node, timeout_sec=0.2)

                recovery_motion()
                publish_state(0)  # ⭐ 추가

                stop_requested = False
                recovery_requested = False
                print("🔄 Recovery 완료 → 재시작")
                continue

    rclpy.shutdown()


if __name__ == "__main__":
    main()
