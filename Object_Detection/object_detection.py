import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class RealSenseColorDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_color_detection')
        self.publisher_ = self.create_publisher(String, 'color_detection', 10)
        self.image_publisher_ = self.create_publisher(Image, 'color_image', 10)
        self.block_publisher = self.create_publisher(Pose, 'block_pose', 10)
        self.bin_publisher = self.create_publisher(Pose, 'bin_pose', 10)

        self.bridge = CvBridge()

        # ✅ 初始化 RealSense 相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ✅ 相机内参
        self.fx, self.fy = 615.0, 615.0
        self.cx, self.cy = 320, 240

        # ✅ 记录识别到的积木块 & 垃圾桶
        self.blocks = {}
        self.bins = {}

        self.timer = self.create_timer(0.1, self.process_frames)

    def process_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # ✅ 1. 预处理
        color_image = cv2.GaussianBlur(color_image, (5, 5), 0)
        hsv_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # ✅ 2. 颜色范围（积木块 + 垃圾桶）
        COLOR_RANGES = {
            'red': (np.array([0, 80, 50]), np.array([10, 255, 255])),
            'yellow': (np.array([20, 150, 100]), np.array([35, 255, 255]))
        }

        COLOR_MAP = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255)
        }

        detection_result = []
        for color, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv_img, lower, upper)

            # ✅ 形态学操作：减少噪声
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                center_x, center_y = int(rect[0][0]), int(rect[0][1])

                # ✅ 计算深度
                depth_values = depth_image[center_y-5:center_y+5, center_x-5:center_x+5].astype(float)
                depth_values = depth_values[depth_values > 0]  # 过滤无效深度
                if len(depth_values) == 0:
                    continue
                depth_value = np.median(depth_values) / 1000.0

                # ✅ 计算 3D 坐标
                X, Y, Z = self.pixel_to_world(center_x, center_y, depth_value)
                Z = np.sqrt((depth_value**2 - Y**2) - X**2 ) #########################################
                # ✅ 物体分类
                if area < 5000 and Z < 0.3:  # **积木块**
                    self.detect_blocks(color, X, Y, Z)
                elif area > 5000 and Z > 0.5:  # **垃圾桶**
                    self.detect_bins(color, X, Y, Z)

                # ✅ 画边界框
                cv2.drawContours(color_image, [box], 0, COLOR_MAP[color], 2)
                cv2.putText(color_image, f"{color}: ({X:.2f}, {Y:.2f}, {Z:.2f})m",
                            (center_x, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_MAP[color], 2)

        # ✅ 发送图像
        img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher_.publish(img_msg)

        # ✅ 显示 OpenCV 画面
        cv2.imshow("Color Detection", color_image)
        cv2.waitKey(1)

    def detect_blocks(self, color, X, Y, Z):
        """ 检测到积木块，记录并放入篮子 """
        if color not in self.blocks:
            self.blocks[color] = []
        self.blocks[color].append((X, Y, Z))

        # 发送抓取指令
        self.publish_task(self.block_publisher, X, Y, Z, color)

        self.get_logger().info(f"Collected {color} block at {X:.2f}, {Y:.2f}, {Z:.2f} and placed in basket.")

    def detect_bins(self, color, X, Y, Z):
        """ 记录垃圾桶的位置 """
        self.bins[color] = (X, Y, Z)
        self.get_logger().info(f"Detected {color} bin at {X:.2f}, {Y:.2f}, {Z:.2f}.")

        # 发送垃圾桶位置
        self.publish_task(self.bin_publisher, X, Y, Z, color)

    def publish_task(self, publisher, X, Y, Z, color):
        """ 发送任务信息 """
        msg = Pose()
        msg.position.x = X
        msg.position.y = Y
        msg.position.z = Z
        msg.orientation.w = 1.0  # 默认方向
        publisher.publish(msg)

    def pixel_to_world(self, u, v, depth):
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def shutdown(self):
        self.pipeline.stop()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
