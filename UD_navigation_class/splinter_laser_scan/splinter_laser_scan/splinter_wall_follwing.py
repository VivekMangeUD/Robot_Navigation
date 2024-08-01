
# Contributors:
# ----------------

# Vivek Mange
# Ismail Lnu
# Ravi Teja Kolli



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos
import random
import time
from sensor_msgs.msg import LaserScan

min_dist = 0.4
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # print('1')
        self.publisher_ = self.create_publisher(Twist, '/zelda/cmd_vel', 10)
        self.forward_flag = True
        self.Rotate_flag = False
        self.check_front = True

        ####### Reactor
        self.subscription = self.create_subscription(
            LaserScan,
            '/zelda/scan',
            self.scan_callback,
            qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning



    def scan_callback(self, msg: LaserScan):
        # print('Inside callback')
        ranges = msg.ranges
        all_min_check = min(ranges)
        if all_min_check <= 0.1:
            self.publish_cmd_vel(0.0,0.0)
            print('emergency stop')
            self.forward_flag = False
        # print(length_range)

        check_list = ranges[1:100] + ranges[len(ranges)-100: len(ranges)-1]
        min_value_check_list_location = ((ranges.index(min(check_list))))
        print('min_value_check_list_location ---', ranges[min_value_check_list_location])

        if self.forward_flag:
            print('forward flag is true --- moving forward')
            self.publish_cmd_vel(0.1, 0.0)

        if self.check_front:
            if ranges[min_value_check_list_location] <= min_dist:
                print('something detected in front cone')
                self.forward_flag = False
                self.check_front = False
                self.Rotate_flag = True

        if self.Rotate_flag:
            print('Roattoin flag is True, Now will rotatae')
            if ranges[250] <= min_dist:
                # print(ranges[250])
                print('here')
                print('Min distance at 250 is 0.5 -- stop roattaion')
                self.publish_cmd_vel(0.0,0.0)
                self.forward_flag = False
                # self.check_front = True
                self.left = True
                self.Rotate_flag = False

            else:
                print('Rotate the robot till min dist at 250 is 0.5 -- Rotating')
                self.publish_cmd_vel(0.0,-0.2)
                print(ranges[250])
                print('done rotation')
                # self.forward_flag = False


        if self.left:
            print('here')
            # if ranges[250] == min_dist:
            #     self.publish_cmd_vel(0.1,0.0)
            if (min_dist-0.08) <ranges[250] < (min_dist+0.08):
                print('2 : ', ranges[250])
                self.publish_cmd_vel(0.1,0.0)
            elif (min_dist+0.08) <ranges[250] < (min_dist+0.15):
                print('range 250 is les than or greater than : ', min_dist+ 0.08)
                self.publish_cmd_vel(0.1,0.4)
            elif (min_dist+0.2) <ranges[250]:
                print('range 250 is gretaer than : ', min_dist+ 0.2)
                self.publish_cmd_vel(0.1,0.5)
            elif ranges[250] < min_dist:
                print('3 : ', ranges[250])          
                self.publish_cmd_vel(0.1,-0.2)
            else:
                print('no ang z ---')
                            

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
