#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # Wiadomość pozycji żółwia
import cv2
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.current_pose = None  # Przechowuje aktualną pozycję żółwia

        self.window_name = "Robot Controller"
        self.point = None
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.create_timer(0.1, self.publish_command)

    def pose_callback(self, msg):
        """Callback, który odbiera dane pozycji żółwia."""
        self.current_pose = msg  # Aktualizujemy bieżącą pozycję

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)

    def publish_command(self):
        if self.point:
            command = Twist()
            # Podział na cztery strefy:
            mid_x, mid_y = 256, 256  # Środek okna
            if self.point[1] < mid_y:  # Góra
                if self.point[0] < mid_x:  # Lewa góra
                    command.angular.z = 1.0  # Obrót w lewo
                else:  # Prawa góra
                    command.linear.x = 1.0  # Ruch do przodu
            else:  # Dół
                if self.point[0] < mid_x:  # Lewa dół
                    command.linear.x = -1.0  # Ruch do tyłu
                else:  # Prawa dół
                    command.angular.z = -1.0  # Obrót w prawo

            self.publisher.publish(command)

        # Wyświetlanie GUI
        img = np.zeros((512, 512, 3), dtype=np.uint8)
        # Linie podziału
        cv2.line(img, (0, 256), (512, 256), (0, 255, 0), 1)  # Linia pozioma
        cv2.line(img, (256, 0), (256, 512), (0, 255, 0), 1)  # Linia pionowa

        # Dodawanie podpisów
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)
        thickness = 1

        # RUCH LEWO
        cv2.putText(img, 'RUCH LEWO', (50, 50), font, font_scale, color, thickness, cv2.LINE_AA)

        # RUCH GORA
        cv2.putText(img, 'RUCH GORA', (300, 50), font, font_scale, color, thickness, cv2.LINE_AA)

        # RUCH TYŁ
        cv2.putText(img, 'RUCH TYL', (50, 300), font, font_scale, color, thickness, cv2.LINE_AA)

        # RUCH PRAWO
        cv2.putText(img, 'RUCH PRAWO', (300, 300), font, font_scale, color, thickness, cv2.LINE_AA)

        # Wyświetlanie aktualnych współrzędnych żółwia
        if self.current_pose:
            position_text = f"X: {self.current_pose.x:.2f}, Y: {self.current_pose.y:.2f}, Theta: {self.current_pose.theta:.2f}"
            cv2.putText(img, position_text, (10, 490), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Kliknięcie myszki
        if self.point:
            point_color = (0, 0, 255)
            cv2.circle(img, self.point, 5, point_color, -1)

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

