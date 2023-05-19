#!/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from enum import IntEnum


SPECIAL_BUTTON = 11


class JoystickState(IntEnum):
    XBOX = 0
    VR = 1


class NesfrVRJoySwitch(Node):

    def __init__(self):
        super().__init__('joy_switch')

        self.state = JoystickState.VR
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.check_xbox_joy)

        self.subscription_xbox = self.create_subscription(
            Joy,
            'xbox_joy',
            self.listener_callback_xbox,
            10)
        self.subscription_xbox  # prevent unused variable warning

        self.subscription_vr = self.create_subscription(
            Joy,
            'vr_joy',
            self.listener_callback_vr,
            10)
        self.subscription_vr  # prevent unused variable warning

        self.xbox_joy_off = False
        self.last_xbox_joy_stamp = rclpy.time.Time(clock_type=rclpy.clock.ClockType.ROS_TIME)

        self.last_button = -1
        self.last_stamp = rclpy.time.Time(clock_type=rclpy.clock.ClockType.ROS_TIME)

        self.xbox_connected = False

    def is_click(self, button):
        ret = False
        stamp = self.get_clock().now()
        duration = stamp - self.last_stamp
        if button == 0 and self.last_button == 1 and (duration.nanoseconds/1e9) < 1:
            ret = True

        self.last_button = button
        self.last_stamp = self.get_clock().now()

        return ret

    def check_xbox_joy(self):
        duration = self.get_clock().now() - self.last_xbox_joy_stamp

        if (duration.nanoseconds/1e9) > 2.0:  # No input from Xbox
            if self.xbox_connected:
                self.get_logger().warning('No input from XBox in 2 seconds')
            self.xbox_connected = False
            self.state = JoystickState.VR
        else:
            if self.xbox_joy_off:  # Turn off Xbox Inputs by Special Key
                self.state = JoystickState.VR
            else:
                self.state = JoystickState.XBOX

    def listener_callback_xbox(self, msg):
        #   self.get_logger().info('xbox: {}'.format(msg))
        if not self.xbox_connected:
            self.get_logger().warning('XBox Connected')
        self.xbox_connected = True

        if self.is_click(msg.buttons[SPECIAL_BUTTON]):
            self.xbox_joy_off = not self.xbox_joy_off
            print("self.xbox_joy_off={}".format(self.xbox_joy_off))
            if self.xbox_joy_off:  # Turn off Xbox Inputs by Special Key
                self.state = JoystickState.VR
            else:
                self.state = JoystickState.XBOX
            self.get_logger().info("JoystickState={}".format(int(self.state)))
        self.last_xbox_joy_stamp = self.get_clock().now()

        if self.state == JoystickState.XBOX:
            #
            # This is workaround against the issue #1 of XBox controller firmware v3.1.1221.0
            # initial values are non-zero even without any inputa
            #
            if msg.axes[0] == 1.0 and msg.axes[1] == 1.0 and msg.axes[2] == 1.0 and msg.axes[3] == 1.0:
                self.get_logger().info('xbox: {}'.format(msg))
                self.get_logger().fatal('These input values cannot be generated from normal controllers. System is ignoring them.')
                pass
            else:
                self.publisher_.publish(msg)
        else:
            pass

    def listener_callback_vr(self, msg):
        #   self.get_logger().info('vr: {}'.format(msg))

        if self.state == JoystickState.VR:
            self.publisher_.publish(msg)
        else:
            pass

def main(args=None):
    rclpy.init(args=args)

    node = NesfrVRJoySwitch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' shutting down by KeyboardInterrupt')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
