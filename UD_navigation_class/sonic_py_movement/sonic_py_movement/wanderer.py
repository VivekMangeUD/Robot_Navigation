# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Contributors:
# ----------------

# Vivek Mange


# Code flow
# move forward until bump, If bump detected, Go back till you hit the back limit and then turn the robot randomly between -180 to 180
# Repeate


import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import HazardDetection
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # print('1')
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            '/splinter/hazard_detection',
            self.wanderer_callback,
            qos.qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Twist, '/splinter/cmd_vel', 10)
        self.timer_period = 0.2  # seconds
        self.forward_flag = True
        self.Rotate_flag = False
        self.check_back_limit_flag = False
        self.subscription  # prevent unused variable warning


    def wanderer_callback(self, msg: HazardDetectionVector):
        if self.forward_flag:
            self.publish_cmd_vel(0.2, 0.0)
            # print("moving forward")
        # print('1')
        for key in msg.detections:
            # print('2')
            if key.type == HazardDetection.BUMP:
                # print('bump')
                # print('stop')
                self.forward_flag = False
                # print("funtion called to bummp")
                self.check_back_limit_flag = True
                # print('check bump flag ----  True')
            
        if self.Rotate_flag:
            # print('rotate robot')
            for i in range(int(random.uniform(5, 15))):
                # print('i --  for loop  ---')
                self.publish_cmd_vel(0.00, random.uniform(-3.00, 3.00))
                self.forward_flag = True
                # print('Forward flag =---------True')
                self.Rotate_flag = False
                # print('Rotate flag =---------False')



        if self.check_back_limit_flag:
            # print('3')
            for bump_back_limit in msg.detections:
                # print('4')
                if bump_back_limit.type == HazardDetection.BACKUP_LIMIT:
                    # print('5')
                    self.publish_cmd_vel(0.0, 0.0)
                    # print('Robot stop')
                    self.check_back_limit_flag = False
                    # print('check bump flag ----  Flase')
                    self.Rotate_flag = True
                    # print('Rotate flag =---------True')

                else:
                    # print('6')
                    self.publish_cmd_vel(-0.01, 0.0)
                    # print('Move back')



    def publish_cmd_vel(self, speeds, angles):
        run = Twist()
        run.linear.x = speeds
        run.angular.z = angles
        self.publisher_.publish(run)
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
