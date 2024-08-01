# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import cv2
import imutils
import time
from geometry_msgs.msg import Twist
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from rclpy import qos
from nav_msgs.msg import Odometry

import math
from tf_transformations import euler_from_quaternion

from irobot_create_msgs.action import RotateAngle
from rclpy.action import ActionClient

#### TODO
'''
call ros2 service inside the ros py file
- Use the below name and type 
ros2 service call /zelda/reset_pose irobot_create_msgs/srv/ResetPose
---------------------------------------------------------------------------------------
Use drive distance action client to go to this distance to hit the ball
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"


Use the below action client to go to the base position after the end of the task

ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true,goal_pose:{pose:{position:{x: 1,y: 0.2,z: 0.0}, orientation:{x: 0.0,y: 0.0, z: 0.0, w: 1.0}}}}"

-----------------------------------------------

'''

#### CV2 ball tracking Params
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())
# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
# greenLower = (120, 35, 75)
# greenUpper = (145, 47, 100)
pts = deque(maxlen=args["buffer"])


ball_hit = np.zeros((1,3))
green_ball_hit_count = 0
go_for_blue = False
go_for_green = True

class ImageSubscriber(Node):
    # print('here 1')
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """ Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
            
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        # print('here 2')
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/video_frames',  
        #     self.listener_callback,
        #     10)
        self.current_x_pos = 0
        self.current_y_pos = 0
        self.ball_hit = np.zeros((1,3))
        self.green_ball_hit_count = 0
        self.go_for_blue = False
        self.go_for_green = False
        self.stop_counter = 0
        self.initial_flag = True
        self.reached_1st_ball = False

        self.second_initiate = False
        self.reached_2nd_ball = False
        self.rotate_towards_y = True

        self.second_initiate = False
        self.reached_2nd_ball = False
        self.rotate_towards_y = True
        self.move_y = self.align_y = self.reach_2nd_green = False
        self.yaw = 0



        self.thrird_initaite = False
        self.rotate_till_blue_0 = False
        self.move_blue_x = False
        self.roatte_tohit_blue = False



        self.go_home = False
        self.align_final_y = False
        self.reach_x = False
        self.align_final_x = False
        self.go_zero = False



        # self.action_client = ActionClient(self, RotateAngle, '/zelda/rotate_angle')

        self.action_client = ActionClient(self, RotateAngle, '/sonic/rotate_angle')

        # self.publisher_ = self.create_publisher(Twist, '/zelda/cmd_vel', 10)
        # self.subscription = self.create_subscription(
        #         Image,
        #         '/camera/color/image_raw',  
        #         self.listener_callback,
        #         10)
        # self.subscription2 = self.create_subscription(
        #         Odometry,
        #         '/zelda/odom',  
        #         self.odom_callback,
        #         qos.qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Twist, '/sonic/cmd_vel', 10)
        self.subscription = self.create_subscription(
                Image,
                '/camera/color/image_raw',  
                self.listener_callback,
                10)
        self.subscription2 = self.create_subscription(
                Odometry,
                '/sonic/odom',  
                self.odom_callback,
                qos.qos_profile_sensor_data)
        # print('here 3')
        self.subscription # prevent unused variable warning
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def odom_callback(self, data:Odometry):
        self.current_x_pos = data.pose.pose.position.x
        self.current_y_pos = data.pose.pose.position.y
        self.current_orientation_x = data.pose.pose.orientation.x
        self.current_orientation_y = data.pose.pose.orientation.y
        self.current_orientation_z = data.pose.pose.orientation.z
        self.current_orientation_w = data.pose.pose.orientation.w

        # quaternion = data.pose.pose.orientation
        # print(quaternion)
        quat_msg = euler_from_quaternion([self.current_orientation_x, self.current_orientation_y, self.current_orientation_z, self.current_orientation_w])
        # euler = tf2_py.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        roll, pitch, yaw = quat_msg
        self.current_yaw = math.degrees(yaw)
        


    def listener_callback(self, data:Image):
        # print('here')
        """
        Callback function.
        """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

        print('Initial flag : ',self.initial_flag)
        print('Green ball hit counter : ', self.green_ball_hit_count)


        if self.initial_flag == False:

            if self.green_ball_hit_count == 2:
                print('Two Green ball hits  -  Go for Blue ')
                self.thrird_initaite = True
                self.go_for_blue = False
                self.go_for_green = False
            elif self.green_ball_hit_count == 1:
                print('Align the robot to hit the 2nd green ball ')
                ## TODO -  Get the logic to get the robot to align to the 2nd ball
                ## Use flags to setup 
                #  Setting the flag to go to second green ball True
                self.second_initiate = True
                self.go_for_green = False
            elif self.reached_1st_ball:
                print("Going for 1st Green Ball")
                self.go_for_green = True
                self.go_for_blue = False
        
        else:
            # if self.initial_flag:
            print('First green ball hitting ')
            if self.current_x_pos < 2:
                print('--------- Moving forward -- Go to straight to 1st green ball at (3,0)')
                self.publish_cmd_vel(0.1,0.0)
            else:
                self.initial_flag = False
                self.reached_1st_ball = True
                time.sleep(5)


        if self.second_initiate:
            # Write the logic to go to second green ball here
            #  CHeck the initial flag False condition and try and improve it
            print('Going to second green ball')

            # -----------------------------------------------
            ## Logic -  Read align the robot in direction of increasing y? - How - Think on that 
            # When robot facing in y direction - move until Y is greater than 2.2
            # Rotate the robot in direction of X decresing 
            # When alligned - Move forward till x is greater than 1.5
            #  Stop the robot and trigger the 2nd green ball detection flag
            # ---------------------------------------------------
            
            if self.rotate_towards_y:
                print('1')
                print('yaw : ', self.current_yaw)
                if -87.00> self.current_yaw > -93.00:
                    self.move_y = True
                    self.rotate_towards_y = False
                    print('Rotate done - Now move forward')
                else:
                    print('2')
                    print('Rotating to align between -87 and -93 ')
                    self.publish_cmd_vel(0.0,-0.05)

            if self.move_y:
                print('3')
                if self.current_y_pos > -2.0:
                    print('4')
                    self.publish_cmd_vel(0.1,0.0)  
                    print('Y pos : ', self.current_y_pos)
 
                    print('Moving forward till y is greater than -2.0')
                    # self.publish_cmd_vel()
                else:
                    print('Reached y -2.0')
                    print('Y pos : ', self.current_y_pos)
                    self.align_y = True
                    self.move_y = False
            if self.align_y:
                print(' yaw : ', self.current_yaw)
                print('5')
                if -178.00> self.current_yaw > -182.00:
                    self.reach_2nd_green = True
                    self.align_y = False
                    print('Rotationn done, Move forward and till green ball')
                else:
                    print('2')
                    print('Rottate to align between -178 and -182')
                    self.publish_cmd_vel(0.0,-0.05)
            if self.reach_2nd_green:
                print('6')
                if self.current_x_pos>2:
                    print('7')
                    print('-- Moving forward till x less than 2.0')
                    print('x Pose : ', self.current_x_pos)
                    self.publish_cmd_vel(0.1,0.0)
                else:
                    self.reach_2nd_green = False
                    self.publish_cmd_vel(0.0,0.0)
                    print('Reach 2nd green, stop the robot')
                    self.second_initiate = False
                    print('second initiate is False')
                    print('wait -- 5 sec')
                    time.sleep(5)
                    self.go_for_green = True
                    

        if self.thrird_initaite:
            # Write the logic to go to second green ball here
            #  CHeck the initial flag False condition and try and improve it
            print('Going to blue ball')

            #------- Logic


            if self.rotate_till_blue_0:
                print('1')
                print('yaw : ', self.current_yaw)
                if 2> self.current_yaw > -2:
                    self.move_blue_x = True
                    self.rotate_till_blue_0 = False
                    print(' align done ---- between +2 and -2')
                else:
                    print('alligning between +2 and -2')
                    self.publish_cmd_vel(0.0,-0.05)

            if self.move_blue_x:
                print('3')
                if self.current_x_pos < 2.0:
                    print('4')
                    self.publish_cmd_vel(0.1,0.0)
                    print(' x pos : ', self.current_x_pos)
                    print(' moving till x is greater than 2.0')
                else:
                    print(' reached x gretaer than 2.0')
                    print('x pos : ', self.current_x_pos)
                    self.roatte_tohit_blue = True
                    self.move_blue_x = False
            if self.roatte_tohit_blue:
                print('5')
                if 92> self.current_yaw > 88:
                    self.roatte_tohit_blue = False
                    print('reached Blue ball - Turn go for blue True')
                    print('thrird initiate is False')
                    self.publish_cmd_vel(0.0,0.0)
                    self.go_for_green = True
                    self.thrird_initaite = False
                    # self.go_home = True
                    # self.align_final_y = True

                else:
                    print('yaw : ', self.current_yaw)
                    print('2')
                    print('Rotating till angle is between 92 and 88 ')
                    self.publish_cmd_vel(0.0,-0.05)            


        if self.go_home:
            # self.go_home = True
            self.go_home_def()


        if self.go_for_green:
            if self.green_ball_hit_count == 2:
                print('going for blue ball')

                self.greenLower = (30, 50, 8)
                self.greenUpper = (85, 255, 255)
            else:
                print("Going to ggreen  ball and have grreen maskk")
                self.greenLower = (98, 6, 100)
                self.greenUpper = (127, 45, 75)

            current_frame = self.br.imgmsg_to_cv2(data)
            frame = current_frame
            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame
            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                print(' Error:  No Frames')
            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
            # hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask_green for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask_green
            mask_green = cv2.inRange(hsv, greenLower, greenUpper)
            mask_green = cv2.erode(mask_green, None, iterations=2)
            mask_green = cv2.dilate(mask_green, None, iterations=2)
            # find contours in the mask_green and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask_green, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                
                print('detected X : ',x,' Y : ',y)

                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                '''
                x -----> 0, 420
                    0    420
                y
                | 0
                |
                | 350

                '''

                # # Target X directions 200(left side in front) to 400(right side in front)
                ## Get the robot to x (290 and 310)
                ## Y range ( 300 - 100 )
                ## target get Y between (300-330)
                # print(current_frame.shape)
                image_x_shape, image_y_shape,_ = (frame.shape)
                image_center_x = image_x_shape/2
                image_center_y = image_y_shape/2    
                # print('frame shape : ', frame.shape)
                # print('image_center_x : ', image_center_x, '  Image center y : ', image_center_y)
                # print('image_center_x + 20 : ', image_center_x +20)
                # print('image_center_x  - 20 : ', image_center_x - 20)
                x = round(x)
                y = round(y)
                # print('x : ', x)

                if 100 < y <370 :
                    print('Y in range')
                    if 300<=y<370:
                        print(' no forward move now ')
                        # self.publish_cmd_vel(0.0,0.0)
                        if 200 < x < 400:
                            print(' x inside 200 and 400 , value ', x)
                            print('check the math')
                            if 270<x<330:
                                print('stop to hit')
                                ## Uncomment 0.0 to let the robot move
                                self.stop_counter +=1
                                print('Stop counter  ------------------- : ', self.stop_counter)
                                if self.stop_counter == 20:
                                    # self.publish_cmd_vel(0.0, 0.0)
                                    self.hit_ball_process()
                                    self.green_ball_hit_count += 1
                                    self.stop_counter = 0
                                    self.go_for_green = False
                                    if self.green_ball_hit_count ==3:
                                        self.go_for_green = False
                                        
                                        self.go_home = True
                                        self.align_final_y = True
                            if 200 <x <270:
                                print('Rotate left')
                                self.stop_counter = 0
                                self.publish_cmd_vel(0.0,0.02)
                            elif 330<x< 400:
                                self.stop_counter = 0
                                print('Rotate right')
                                self.publish_cmd_vel(0.0,-0.02)
                            
                        else:
                            self.publish_cmd_vel(0.0,0.05)
                    elif 200<=y<300:
                        print('ball is close, move forward with slow speed 0.05')
                        self.publish_cmd_vel(0.05,0.0)
                    elif 100<y<200:
                        self.publish_cmd_vel(0.1,0.0)
                        print('ball is far away, move forward with high speed')
                else:
                    print('move forward to align the robot in y range 0.1')
                    self.publish_cmd_vel(0.1,0.0)
            else:
                self.publish_cmd_vel(0.0,0.1)

            # update the points queue

            pts.appendleft(center)
            
                        
            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue
                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

            # '''
            # x -----> 0, 420
            #     0    420
            # y
            # | 0
            # |
            # | 350

            # '''
            # print(current_frame.shape)
            # image_center_x, image_center_y,_ = current_frame.shape
            # if image_center_x +20 < x: 
            #     self.publish_cmd_vel(0.0, 0.2)
            # elif x < image_center_x:
            #     self.publish_cmd_vel(0.0, -0.2)



            ## show the frame to our screen
            cv2.imshow("Frame", frame)
            cv2.imshow('green mask ', mask_green)
            cv2.imshow('HSV frame', hsv)
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                print(' Press 1 to break')


    def go_home_def(self):
        print('Going home')

        if self.align_final_y:
            print(' yaw : ', self.current_yaw)
            print('5')
            if -178.00> self.current_yaw > -182.00:
                self.reach_x = True
                self.align_final_y = False
                print(' -- Rotationn done, Move forward and till green ball')
            else:
                print('2')
                print(' --- Rottate to align between -178 and -182')
                self.publish_cmd_vel(0.0,-0.05)
        if self.reach_x:
            if self.current_x_pos < 0.1:
                self.publish_cmd_vel(0.0,0.0)
                self.align_final_x = True
                self.reach_x = False
                print('reached at x = 0')
            else:
                self.publish_cmd_vel(0.1,0.0)
                print('moving forward till x is 0')
        if self.align_final_x:
            if 92> self.current_yaw > 88:
                self.align_final_x = False
                self.go_zero = True
                print('aligned in x - to to 0 , 0')
                self.publish_cmd_vel(0.0,0.0)
            else:
                print('yaw : ', self.current_yaw)
                print('2')
                print('Rotating till angle is between 92 and 88 ')
                self.publish_cmd_vel(0.0,-0.05)   

        if self.go_zero:
            if self.current_y_pos>-0.1:
                self.publish_cmd_vel(0.0,0.0)
                print('Reached home')
                self.go_zero = False
                self.go_home = False
            else:
                self.publish_cmd_vel(0.1,0.0)
                print('moving in y - going to 0, 0')
    
    def hit_ball_process(self):
        print('Initaite the hit process ----')
        for i in range(0,30):
            self.publish_cmd_vel(0.1,0.0)
            print('--- moving forward --- ')


    def publish_cmd_vel(self, speeds, angles):
        run = Twist()
        run.linear.x = speeds
        run.angular.z = angles
        self.publisher_.publish(run)
        time.sleep(0.1)

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()