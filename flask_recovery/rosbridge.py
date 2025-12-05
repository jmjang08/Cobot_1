import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

ROBOT_ID = "dsr01"

class RosBridge(Node):
    def __init__(self):
        super().__init__("ros_web_bridge", namespace=ROBOT_ID)

        self.socketio = None

        # 웹 → ROS 퍼블리셔
        self.start_pub = self.create_publisher(Bool, "/start_signal", 10)
        self.mode_pub  = self.create_publisher(Int32, "/mode_select", 10)
        self.stop_pub  = self.create_publisher(Bool, "/stop_signal", 10)
        self.recovery_pub = self.create_publisher(Bool, "/recovery_signal", 10) # Recovery 퍼블리셔 정의
        self.end_pub = self.create_publisher(Bool, "/end_signal", 10)
        
        # ROS → 웹 구독자
        self.create_subscription(
            Int32,
            "/progress_state",
            self.cb_progress,
            10
        )

        print("RosBridge Ready.")

    # START publish
    def publish_end(self):
        msg = Bool()
        msg.data = True
        self.end_pub.publish(msg)
        self.get_logger().info("Publish /end_signal TRUE")
    
    def publish_start(self, flag: bool):
        msg = Bool()
        msg.data = flag
        self.start_pub.publish(msg)
        self.get_logger().info("Publish /start_signal TRUE")

    # MODE publish
    def publish_mode(self, mode: int):
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Publish /mode_select {mode}")

    # ⭐ STOP publish (함수 인수를 제거하여 app.py와 통일)
    def publish_stop(self):
        msg = Bool()
        msg.data = True # STOP은 항상 True 발행
        self.stop_pub.publish(msg)
        self.get_logger().info("Publish /stop_signal TRUE")

    # 🛑 RECOVERY publish 함수 추가 🛑
    def publish_recovery(self):
        msg = Bool()
        msg.data = True # RECOVERY는 항상 True 발행
        self.recovery_pub.publish(msg)
        self.get_logger().info("--- PUBLISHED: /recovery_signal TRUE ---") # Recovery 로그 추가

    # ROS → Web emit
    def cb_progress(self, msg: Int32):
        state = msg.data
        print(f"[ROS] /progress_state = {state}")

        if self.socketio:
            self.socketio.emit("progress_update", {"state": state})
            print(f"[ROS → Web] progress_update {state}")
        else:
            print("⚠ socketio not connected")
