from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import rclpy

from rosbridge import RosBridge

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

# ROS2 INIT
rclpy.init()

# 브릿지 생성
ros_node = RosBridge()
ros_node.socketio = socketio  # 웹 emit 연결

# ROS spin thread
ros_thread = threading.Thread(
    target=rclpy.spin,
    args=(ros_node,),
    daemon=True
)
ros_thread.start()


@app.route('/')
def index():
    return render_template("index.html")

# 웹 → ROS2 (START)
@socketio.on('start_signal')
def handle_start_signal(msg):
    ros_node.publish_start(True)
    print("[Web → ROS] start_signal TRUE")

# 웹 → ROS2 (mode 1/2/3)
@socketio.on('mode_select')
def handle_mode_select(data):
    mode = data["mode"]
    ros_node.publish_mode(mode)
    print("[Web → ROS] mode =", mode)
    
# 웹 → ROS2 (STOP)
@socketio.on('stop_signal')
def handle_stop_signal(msg):
    # ⭐ 수정: publish_stop 함수는 인수를 받지 않도록 통일했습니다.
    ros_node.publish_stop() 
    print("[Web → ROS] stop_signal TRUE")

# ⭐ 웹 → ROS2 (RECOVERY) 핸들러 추가 ⭐
@socketio.on('recovery_signal')
def handle_recovery_signal(msg):
    # Recovery 신호를 받으면 ROS Bridge의 publish_recovery 함수 호출
    ros_node.publish_recovery()
    print("[Web → ROS] recovery_signal TRUE")
    
# 웹 → ROS2 (END)
@socketio.on('end_signal')
def handle_end_signal(msg):
    ros_node.publish_end()
    print("[Web → ROS] end_signal TRUE")

    
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
