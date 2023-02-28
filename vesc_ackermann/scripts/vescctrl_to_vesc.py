#!/usr/bin/env python

import rospy
from vesc_msgs.msg import VescCtrlStamped
from std_msgs.msg import Float64,Int64

class VescControl:
    def __init__(self):
        self.speed_to_erpm_gain = rospy.get_param('/vesc/speed_to_erpm_gain')
        self.speed_to_erpm_offset = rospy.get_param('/vesc/speed_to_erpm_offset')
        self.steering_angle_to_servo_gain = rospy.get_param('/vesc/steering_angle_to_servo_gain')
        self.steering_angle_to_servo_offset = rospy.get_param('/vesc/steering_angle_to_servo_offset')

        self.vescctrl_sub = rospy.Subscriber('/vesc/ctrl',VescCtrlStamped,self.vescctrl_callback)
        self.mode_pub = rospy.Publisher('/vesc/commands/motor/mode',Int64,queue_size=1)
        self.current_pub = rospy.Publisher('/vesc/commands/motor/raw_current', Float64, queue_size=1)
        self.brake_pub = rospy.Publisher('/vesc/commands/motor/raw_brake', Float64, queue_size=1)
        self.speed1_pub = rospy.Publisher('/vesc/commands/motor/raw_speed1', Float64, queue_size=1)
        self.speed2_pub = rospy.Publisher('/vesc/commands/motor/raw_speed2', Float64, queue_size=1)
        self.speed3_pub = rospy.Publisher('/vesc/commands/motor/raw_speed3', Float64, queue_size=1)
        self.speed4_pub = rospy.Publisher('/vesc/commands/motor/raw_speed4', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/vesc/commands/servo/raw_position', Float64, queue_size=1)
    
    def vescctrl_callback(self, ctrlmsg):
        mode = ctrlmsg.control.mode
        duty = ctrlmsg.control.duty_cycle
        current = ctrlmsg.control.current
        brake = ctrlmsg.control.brake
        speed1 = ctrlmsg.control.speed1 * self.speed_to_erpm_gain + self.speed_to_erpm_offset
        speed2 = ctrlmsg.control.speed2 * self.speed_to_erpm_gain + self.speed_to_erpm_offset
        speed3 = ctrlmsg.control.speed3 * self.speed_to_erpm_gain + self.speed_to_erpm_offset
        speed4 = ctrlmsg.control.speed4 * self.speed_to_erpm_gain + self.speed_to_erpm_offset
        position = ctrlmsg.control.position
        servo = ctrlmsg.control.servo * self.steering_angle_to_servo_gain + self.steering_angle_to_servo_offset
        self.servo_pub.publish(servo)
        self.mode_pub.publish(mode)
        self.brake_pub.publish(brake)
        self.current_pub.publish(current)
        self.speed1_pub.publish(speed1)
        self.speed2_pub.publish(speed2)
        self.speed3_pub.publish(speed3)
        self.speed4_pub.publish(speed4)


rospy.init_node('vescctrl_to_vesc')
vesccontrol = VescControl()
rospy.spin()
