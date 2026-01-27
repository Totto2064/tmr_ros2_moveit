#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


class JoyToServo(Node):
    def __init__(self):
        super().__init__('joy_to_servo')
        
        # パラメータ宣言
        self.declare_parameter('command_frame', 'flange')
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_linear_z_up', 5)
        self.declare_parameter('axis_linear_z_down', 2)
        self.declare_parameter('axis_angular_x', 3)
        self.declare_parameter('axis_angular_y', 4)
        self.declare_parameter('axis_angular_z', -1)
        self.declare_parameter('button_angular_z_pos', 5)  # R1 → ヨー正方向
        self.declare_parameter('button_angular_z_neg', 4)  # L1 → ヨー負方向
        self.declare_parameter('linear_scale', 0.1)
        self.declare_parameter('angular_scale', 0.2)
        self.declare_parameter('publish_rate', 50.0)
        
        # パラメータ取得
        self.command_frame = self.get_parameter('command_frame').value
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_linear_z_up = self.get_parameter('axis_linear_z_up').value
        self.axis_linear_z_down = self.get_parameter('axis_linear_z_down').value
        self.axis_angular_x = self.get_parameter('axis_angular_x').value
        self.axis_angular_y = self.get_parameter('axis_angular_y').value
        self.axis_angular_z = self.get_parameter('axis_angular_z').value
        self.button_angular_z_pos = self.get_parameter('button_angular_z_pos').value
        self.button_angular_z_neg = self.get_parameter('button_angular_z_neg').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 現在のJoy状態
        self.joy_msg = None
        
        # Subscriber / Publisher
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )
        
        # タイマー
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_twist)
        
        # サーボ開始サービスクライアント
        self.start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        self.start_timer = self.create_timer(2.0, self.call_start_servo)
        self.servo_started = False
        
        self.get_logger().info('JoyToServo node started')

    def call_start_servo(self):
        if self.servo_started:
            return
        
        if self.start_servo_client.service_is_ready():
            future = self.start_servo_client.call_async(Trigger.Request())
            future.add_done_callback(self.start_servo_callback)
            self.get_logger().info('Calling /servo_node/start_servo...')
            self.servo_started = True
            self.destroy_timer(self.start_timer)
        else:
            self.get_logger().info('Waiting for /servo_node/start_servo service...')

    def start_servo_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Servo started successfully')
            else:
                self.get_logger().warn(f'Servo start failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def joy_callback(self, msg: Joy):
        self.joy_msg = msg

    def publish_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame
        
        if self.joy_msg is not None:
            axes = self.joy_msg.axes
            buttons = self.joy_msg.buttons
            
            # 線速度
            twist.twist.linear.x = axes[self.axis_linear_x] * self.linear_scale
            twist.twist.linear.y = axes[self.axis_linear_y] * self.linear_scale
            # Z軸: RT(上昇) - LT(下降)
            z_up = (1.0 - axes[self.axis_linear_z_up]) / 2.0   # [1,-1] → [0,1]
            z_down = (1.0 - axes[self.axis_linear_z_down]) / 2.0
            twist.twist.linear.z = (z_up - z_down) * self.linear_scale
            
            # 角速度
            if self.axis_angular_x >= 0:
                twist.twist.angular.x = axes[self.axis_angular_x] * self.angular_scale
            if self.axis_angular_y >= 0:
                twist.twist.angular.y = axes[self.axis_angular_y] * self.angular_scale
            if self.axis_angular_z >= 0:
                twist.twist.angular.z = axes[self.axis_angular_z] * self.angular_scale
            
            # ボタンによるangular.z制御 (R1: 正, L1: 負)
            if self.button_angular_z_pos >= 0 and self.button_angular_z_neg >= 0:
                z_rot = 0.0
                if buttons[self.button_angular_z_pos]:
                    z_rot += 1.0
                if buttons[self.button_angular_z_neg]:
                    z_rot -= 1.0
                twist.twist.angular.z = z_rot * self.angular_scale
        
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
