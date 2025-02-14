import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
from ament_index_python.packages import get_package_share_directory

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Publisher für das Image-Topic
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)

        # Lade das Bild
        package_name = 'robonav'  # Ändere den Paketnamen entsprechend
        image_path = os.path.join(
            get_package_share_directory(package_name), 'worlds', 'Eurobot_map_real_bw_10_p.png'  # Ändere den Pfad entsprechend
        )

        if not os.path.exists(image_path):
            self.get_logger().error(f"Bild {image_path} nicht gefunden!")
            return

        self.bridge = CvBridge()
        self.image = cv2.imread(image_path, cv2.IMREAD_COLOR)  # Lade das Bild als Farb-Image

        if self.image is None:
            self.get_logger().error("Fehler beim Laden des Bildes!")
            return

        # Veröffentliche das Bild periodisch
        self.timer = self.create_timer(1.0, self.publish_image)

    def publish_image(self):
        """Bild als ROS 2 Image-Nachricht veröffentlichen"""
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info("Bild veröffentlicht auf /image_raw")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
