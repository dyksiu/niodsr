#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.window_name = "Robot Controller"
        self.point = None
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.create_timer(0.1, self.publish_command)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)

    def publish_command(self):
        if self.point:
            command = Twist()
            if self.point[1] < 256:  # Kliknięcie powyżej środka
                command.linear.x = 1.0
            else:  # Kliknięcie poniżej środka
                command.linear.x = -1.0
            self.publisher.publish(command)
        # Wyświetlanie GUI
        img = np.zeros((512, 512, 3), dtype=np.uint8)
        cv2.line(img, (0, 256), (512, 256), (0, 255, 0), 1)
        if self.point:
            color = (0, 0, 255) if self.point[1] < 256 else (255, 0, 0)
            cv2.circle(img, self.point, 5, color, -1)
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

