from leap_client.msg import HandInfo, FingerInfo, HandInfoList
from kinova_msgs.msg import JointVelocity
from movo_msgs.msg import JacoCartesianVelocityCmd
from tf.transformations import quaternion_multiply,euler_from_quaternion
import rospy
import const
import numpy as np

class LeapListener:
    def __init__(self,robotName = 'movo'):
        rospy.init_node('leap_listener', anonymous=True)
        self.robot_name = "/" + robotName

        self.input_staleness = 0
        self.anchor_cooldown = 0
        self.leap_input = None

        self.hand_state = None
        self.left_hand_state = None
        self.right_hand_state = None

        self.leap_subscriber = None
        self.left_arm_subscriber = None
        self.right_arm_subscriber = None

        self.left_arm_publisher = None
        self.right_arm_publisher = None
        self.left_gripper_publisher = None
        self.right_gripper_publisher = None

        self.hand_radii = None
        self.run()
    def quaternion_invert(self, q):
        q[3] *= -1
        return q
    def get_hand_diff(self, anchor_data, hand_data):
        res = [0]*3
        if self.left_hand_state is None:
            return res
        res[0] = hand_data.pose.position.x - anchor_data.pose.position.x
        res[1] = hand_data.pose.position.y - anchor_data.pose.position.y
        res[1] *= -1
        res[2] = hand_data.pose.position.z - anchor_data.pose.position.z
        res[2] *= -1
        anchor_q = [    anchor_data.pose.orientation.x,
                        anchor_data.pose.orientation.y,
                        anchor_data.pose.orientation.z,
                        anchor_data.pose.orientation.w
                    ]

        hand_q =    [   hand_data.pose.orientation.x,
                        hand_data.pose.orientation.y,
                        hand_data.pose.orientation.z,
                        hand_data.pose.orientation.w
                    ]
        angle = euler_from_quaternion(quaternion_multiply(self.quaternion_invert(anchor_q),hand_q))
        angle = list(angle)
        angle[2] *= -1
        return res+angle
    def receive_hand_info(self, data):
        self.input_staleness = 0
        self.leap_input = data
    def vector_info(self, vec):
        norm = 0.
        unit = vec
        for i,val in enumerate(vec):
            norm += val*val
        norm = norm**0.5
        if norm == 0:
            return (norm,unit)
        for i in range(len(unit)):
            unit[i] /= norm
        return (norm,unit)

    def create_jaco_pose_vel_cmd(self,cmd_data):
        pose_cmd = JacoCartesianVelocityCmd()
        pose_cmd.x = cmd_data[0]
        pose_cmd.y = cmd_data[1]
        pose_cmd.z = cmd_data[2]
        pose_cmd.theta_x = cmd_data[3]
        pose_cmd.theta_y = cmd_data[4]
        pose_cmd.theta_z = cmd_data[5]
        return pose_cmd
    def send_left_arm_pose_vel_cmd(self,cmd):
        self.left_arm_publisher.publish(cmd)
    def send_right_arm_pose_vel_cmd(self,cmd):
        self.right_arm_pubisher.publish(cmd)
    def anchor_left_hand(self,pos):
        self.left_hand_state = pos
    def anchor_right_hand(self,pos):
        self.right_hand_state = pos
    def receive_pose_info(self, data):
        pass
    def reset_states(self):
        self.left_hand_state = None
        self.right_hand_state = None
        self.leap_input = None
        self.anchor_cooldown = 0
    def run(self):
        self.leap_subscriber = rospy.Subscriber("/leap/hand_info",HandInfoList,self.receive_hand_info,queue_size=10)
        self.left_arm_publisher = rospy.Publisher(self.robot_name + "/left_arm/cartesian_vel_cmd", JacoCartesianVelocityCmd,queue_size=10)
        rate = rospy.Rate(100)
        self.input_staleness = 0
        dummy_cmd = [0,0,0,0,0,0]
        while not rospy.is_shutdown():
            if self.leap_input is not None:
                for hand in self.leap_input.hands:
                    if (hand.sphere_radius < const.FIST_RADIUS_THRESHOLD):
                        self.anchor_cooldown = const.ANCHOR_COOLDOWN
                    if self.anchor_cooldown > 0:
                        self.anchor_left_hand(hand)
                        self.anchor_cooldown -= 1
                    vals = self.get_hand_diff(self.left_hand_state,hand)
                    norm,unit = self.vector_info(vals[:3])
                    a_norm,a_unit = self.vector_info(vals[3:])
                    print(hand.sphere_radius)
                    print(norm,unit)
                    print("ANGLES",a_norm,a_unit)
                    if a_norm >= const.ANGLE_DEAD_ZONE:
                        a_norm -= const.ANGLE_DEAD_ZONE_DAMPENER
                        for i in range(len(a_unit)):
                            a_unit[i] *= norm*const.ANGLE_SENSITIVITY
                    else:
                        a_unit = [0,0,0]
                    if norm >= const.DEAD_ZONE:
                        norm -= const.DEAD_ZONE_DAMPENER
                        for i in range(len(unit)):
                            unit[i] *= norm*const.SENSITIVITY
                    else:
                        unit = [0,0,0]
                    cmd = self.create_jaco_pose_vel_cmd(unit + a_unit)
                    self.send_left_arm_pose_vel_cmd(cmd)
                    #print(hand.id)
            else:
                print("E")
            if self.input_staleness >= const.INPUT_STALENESS_THRESHOLD:
                self.reset_states()
            self.input_staleness += 1
            rate.sleep()

