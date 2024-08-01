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



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time

## Reactor
from irobot_create_msgs.msg import IrIntensityVector

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/splinter/cmd_vel', 10)
        self.forward_flag = True
        self.Rotate_flag = False

        ####### Reactor
        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/splinter/ir_intensity',
            self.reactor_callback,
            qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        
    def reactor_callback(self, msg: IrIntensityVector):
        value_list = []
        for value in msg.readings:
            value_list.append(value.value)
        if max(value_list) > 200:
            # print('value list at max 300 : ', value_list)
            if value_list.index(max(value_list)) == 6:
                self.rotate_cmd_update('l', 0.2)
            elif value_list.index(max(value_list)) == 5:
                self.rotate_cmd_update('l', 0.4)
            elif value_list.index(max(value_list)) == 4:
                self.rotate_cmd_update('l', 0.6)
            elif value_list.index(max(value_list)) == 3:
                self.rotate_cmd_update('r', -0.6)
            elif value_list.index(max(value_list)) == 2:
                self.rotate_cmd_update('r', -0.6)
            elif value_list.index(max(value_list)) == 1:
                self.rotate_cmd_update('r', -0.4)
            elif value_list.index(max(value_list)) == 0:
                self.rotate_cmd_update('r', -0.2)
        

            # #  Uncomment for submission
            # if value_list.index(max(value_list)) >= 4:
            #     self.rotate_cmd('l')
            # else:
            #     self.rotate_cmd('r')
            # self.forward_flag = False
            # # self.Rotate_flag = True
            # #  Comment end
        else:
            self.publish_cmd_vel(0.1, 0.0)
            # print("moving forward")


    def rotate_cmd_update(self, angle, ang_speed):
        for i in range(int(random.uniform(5, 15))):
            if angle == 'l':
                # print('=------------Rotate Left')
                self.publish_cmd_vel(0.00, ang_speed)
            elif angle == 'r':
                # print('=------------Rotate right')
                self.publish_cmd_vel(0.00, ang_speed)
            self.forward_flag = True
            # print('Forward flag =---------True')
            self.Rotate_flag = False
            # print('Rotate flag =---------False')

# # Uncomment for method 1
#     def rotate_cmd(self, angle):
#         for i in range(int(random.uniform(5, 15))):
#             # print('i --  for loop  ---')
#             if angle == 'l':
#                 # print('=------------Rotate Left')
#                 self.publish_cmd_vel(0.00, random.uniform(0, 3.0))
#             else:
#                 # print('=------------Rotate right')
#                 self.publish_cmd_vel(0.00, random.uniform(0,-3.00))
#             self.forward_flag = True
#             # print('Forward flag =---------True')
#             self.Rotate_flag = False
#             # print('Rotate flag =---------False')
# # Comment End

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
