import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2, csv, os
from datetime import datetime

class RoadInspection(Node):
    def __init__(self):
        super().__init__('road_inspection_node')
        self.bridge = CvBridge()

        # Suscripción a la cámara y odometría
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.latest_pose = (0.0, 0.0)

        # CSV para guardar resultados
        out_dir = os.path.expanduser('~/ros2_ws/data')
        os.makedirs(out_dir, exist_ok=True)
        self.csv_path = os.path.join(out_dir, 'detections.csv')
        new_file = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if new_file:
            self.csv_writer.writerow(['timestamp', 'x', 'y', 'crack_pixels'])

        self.get_logger().info('✅ Road inspection node initialized.')

    def on_odom(self, msg: Odometry):
        self.latest_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Procesamiento clásico con OpenCV
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 80, 180)
        _, thresh = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Métrica simple
        crack_pixels = int(cv2.countNonZero(morph))

        # Overlay visual
        overlay = cv2.addWeighted(frame, 0.8, cv2.cvtColor(morph, cv2.COLOR_GRAY2BGR), 0.5, 0)
        cv2.imshow('Crack Detection Overlay', overlay)
        cv2.waitKey(1)

        # Guardar en CSV
        ts = datetime.utcnow().isoformat()
        x, y = self.latest_pose
        self.csv_writer.writerow([ts, f'{x:.3f}', f'{y:.3f}', crack_pixels])
        self.csv_file.flush()

def main():
    rclpy.init()
    node = RoadInspection()
    rclpy.spin(node)
    node.csv_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
