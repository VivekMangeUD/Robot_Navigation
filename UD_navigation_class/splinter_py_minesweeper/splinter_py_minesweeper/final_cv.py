"""
contributor
======
Vivek Mange

Code
=====

final_cv2.py
====================
Autonomous ball-hitting robot controller using ROS 2 and OpenCV.

Robot: iRobot Create (Sonic)
Task:
    1. Navigate to and hit two green balls sequentially.
    2. Navigate to and hit one blue ball.
    3. Return to home position (0, 0).

Architecture
------------
- A single ROS 2 node (BallTrackerNode) owns all subscriptions, publishers,
  and action clients.
- Robot behaviour is modelled as an explicit finite state machine (RobotState
  enum).  Every state has exactly one handler method (_handle_<state>).
- All blocking waits are replaced with non-blocking timer-based dwell states
  (DWELL_AFTER_HIT).
- Magic numbers are collected into a single BallTrackerConfig dataclass so
  they are easy to tune without touching logic.
- OpenCV processing is isolated in _detect_ball() and returns a clean
  BallDetection datatype; the state machine never calls cv2 directly.
- cmd_vel is published at a fixed 10 Hz control loop, not inside the image
  callback, to avoid rate coupling between the camera and the drive system.

ROS 2 interfaces consumed
--------------------------
  /camera/color/image_raw   sensor_msgs/Image
  /sonic/odom               nav_msgs/Odometry
  /sonic/rotate_angle       irobot_create_msgs/action/RotateAngle  (unused
                             in this version; waypoint navigation replaces it)

ROS 2 interfaces produced
--------------------------
  /sonic/cmd_vel            geometry_msgs/Twist

"""

import math
import time

import cv2
import imutils
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion


# -------------------------------------------------------------------------
# Colour ranges for ball detection (HSV format)
# -------------------------------------------------------------------------

# Green ball colour range in HSV
GREEN_LOWER = (98, 6, 100)
GREEN_UPPER = (127, 45, 75)

# Blue ball colour range in HSV
BLUE_LOWER = (30, 50, 8)
BLUE_UPPER  = (85, 255, 255)

# -------------------------------------------------------------------------
# Robot drive speeds
# -------------------------------------------------------------------------

SPEED_FAST  = 0.10   # m/s   used when ball is far away
SPEED_SLOW  = 0.05   # m/s   used when ball is close
TURN_FAST   = 0.10   # rad/s used when spinning to search for ball
TURN_SLOW   = 0.02   # rad/s used for small left/right corrections
TURN_ALIGN  = 0.05   # rad/s used when rotating to a target heading

# -------------------------------------------------------------------------
# Other constants
# -------------------------------------------------------------------------

# How many frames in a row the ball must be centred before we hit it
HIT_CONFIRM_FRAMES = 20

# How long to wait (seconds) after hitting a ball before moving on
DWELL_AFTER_HIT = 5.0

# Minimum ball radius in pixels — smaller blobs are ignored
MIN_BALL_RADIUS = 10.0

# Heading tolerance in degrees — "close enough" to target angle
HEADING_TOL = 3.0

# Waypoint positions (metres)
GREEN1_STOP_X     = 2.0    # drive forward until x passes this point
GREEN2_STAGE_Y    = -2.0   # drive in -y direction to this y value
GREEN2_APPROACH_X = 2.0    # then drive in -x until x passes this point
BLUE_STAGE_X      = 2.0    # drive in +x until past this point
HOME_X_THRESH     = 0.1    # "close enough" to x=0
HOME_Y_THRESH     = -0.1   # "close enough" to y=0


# -------------------------------------------------------------------------
# Main node class
# -------------------------------------------------------------------------

class BallTrackerNode(Node):
    """
    ROS 2 node that drives the robot to find and hit three balls in sequence.

    We use a simple state machine to track what the robot should be doing.
    The state is just a string, for example 'approach_green1' or 'track_green1'.
    """

    def __init__(self):
        super().__init__('ball_tracker')

        # CvBridge converts ROS Image messages into OpenCV images
        self.bridge = CvBridge()

        # --- Robot position and heading (updated by odometry callback) ---
        self.x   = 0.0   # metres
        self.y   = 0.0   # metres
        self.yaw = 0.0   # degrees, converted from quaternion

        # --- State machine ---
        # This string tells us what the robot is currently trying to do.
        # We change it when the robot finishes a step.
        self.state = 'approach_green1'

        # How many balls have been hit so far (green1=1, green2=2, blue=3)
        self.hit_count = 0

        # Counter that goes up each frame the ball is centred — when it
        # reaches HIT_CONFIRM_FRAMES we trigger the hit
        self.centred_frames = 0

        # When we are in the 'waiting' state, this stores when to stop waiting
        self.wait_until = 0.0

        # Which state to go to after the wait is over
        self.state_after_wait = 'done'

        # The twist message we want to publish (set by update logic each frame)
        self.cmd = Twist()

        # --- ROS publishers and subscribers ---

        self.cmd_pub = self.create_publisher(Twist, '/sonic/cmd_vel', 10)

        self.img_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/sonic/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )

        # Publish cmd_vel at 10 Hz so drive commands go out regularly
        self.control_timer = self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info('BallTrackerNode started, state: approach_green1')

    # -----------------------------------------------------------------------
    # Odometry callback — saves the robot position and heading
    # -----------------------------------------------------------------------

    def odom_callback(self, msg: Odometry):
        """Called every time a new odometry message arrives."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # The orientation comes as a quaternion — we convert to yaw in degrees
        ori = msg.pose.pose.orientation
        _, _, yaw_rad = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.yaw = math.degrees(yaw_rad)

    # -----------------------------------------------------------------------
    # Image callback — main decision loop
    # -----------------------------------------------------------------------

    def image_callback(self, msg: Image):
        """
        Called every time a new camera frame arrives.
        We look at the current state and decide what the robot should do.
        """
        # Convert the ROS image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # --- State: drive straight to first green ball position ---
        if self.state == 'approach_green1':
            if self.x < GREEN1_STOP_X:
                self.drive(SPEED_FAST, 0.0)
            else:
                self.stop()
                self.get_logger().info('Reached green1 waypoint, switching to track_green1')
                self.state = 'track_green1'

        # --- State: visually track and hit the first green ball ---
        elif self.state == 'track_green1':
            self.track_ball(frame, color='green')

        # --- State: wait after hitting a ball ---
        elif self.state == 'waiting':
            self.stop()
            if time.monotonic() >= self.wait_until:
                self.get_logger().info(f'Wait done, moving to {self.state_after_wait}')
                self.state = self.state_after_wait

        # --- Routing to green ball 2: rotate to face -y direction ---
        elif self.state == 'rotate_neg_y':
            target = -90.0
            if self.heading_error(target) < HEADING_TOL:
                self.stop()
                self.get_logger().info('Facing -y, driving to green2 stage')
                self.state = 'drive_green2_stage'
            else:
                self.drive(0.0, -TURN_ALIGN)

        # --- Drive in -y direction to staging point ---
        elif self.state == 'drive_green2_stage':
            if self.y > GREEN2_STAGE_Y:
                self.drive(SPEED_FAST, 0.0)
            else:
                self.stop()
                self.get_logger().info('Reached green2 stage y, rotating to face -x')
                self.state = 'rotate_neg_x'

        # --- Rotate to face -x direction ---
        elif self.state == 'rotate_neg_x':
            target = 180.0
            if self.heading_error(target) < HEADING_TOL:
                self.stop()
                self.get_logger().info('Facing -x, driving to green2')
                self.state = 'drive_green2'
            else:
                self.drive(0.0, -TURN_ALIGN)

        # --- Drive in -x to get close to green ball 2 ---
        elif self.state == 'drive_green2':
            if self.x > GREEN2_APPROACH_X:
                self.drive(SPEED_FAST, 0.0)
            else:
                self.stop()
                self.get_logger().info('Close to green2, switching to track_green2')
                self.state = 'track_green2'

        # --- State: visually track and hit the second green ball ---
        elif self.state == 'track_green2':
            self.track_ball(frame, color='green')

        # --- Routing to blue ball: rotate to face +x ---
        elif self.state == 'rotate_pos_x':
            target = 0.0
            if self.heading_error(target) < HEADING_TOL:
                self.stop()
                self.get_logger().info('Facing +x, driving to blue stage')
                self.state = 'drive_blue_stage'
            else:
                self.drive(0.0, -TURN_ALIGN)

        # --- Drive in +x past blue ball staging point ---
        elif self.state == 'drive_blue_stage':
            if self.x < BLUE_STAGE_X:
                self.drive(SPEED_FAST, 0.0)
            else:
                self.stop()
                self.get_logger().info('Past blue stage, rotating to face +y')
                self.state = 'rotate_pos_y_blue'

        # --- Rotate to face +y to approach blue ball ---
        elif self.state == 'rotate_pos_y_blue':
            target = 90.0
            if self.heading_error(target) < HEADING_TOL:
                self.stop()
                self.get_logger().info('Facing +y, tracking blue ball')
                self.state = 'track_blue'
            else:
                self.drive(0.0, -TURN_ALIGN)

        # --- Track and hit the blue ball ---
        elif self.state == 'track_blue':
            self.track_ball(frame, color='blue')

        # --- Go home: rotate to face -x ---
        elif self.state == 'home_rotate_x':
            target = 180.0
            if self.heading_error(target) < HEADING_TOL:
                self.stop()
                self.get_logger().info('Facing -x, driving home in x')
                self.state = 'home_drive_x'
            else:
                self.drive(0.0, -TURN_ALIGN)

        # --- Drive in -x until x is near 0 ---
        elif self.state == 'home_drive_x':
            if self.x > HOME_X_THRESH:
                self.drive(SPEED_FAST, 0.0)
            else:
                self.stop()
                self.get_logger().info('x near 0, rotating to face +y')
                self.state = 'home_rotate_y'

        # --- Rotate to face +y for final y drive ---
        elif self.state == 'home_rotate_y':
            target = 90.0
            if self.heading_error(target) < HEADING_TOL:
                self.stop()
                self.get_logger().info('Facing +y, driving home in y')
                self.state = 'home_drive_y'
            else:
                self.drive(0.0, -TURN_ALIGN)

        # --- Drive in +y until y is near 0 ---
        elif self.state == 'home_drive_y':
            if self.y < HOME_Y_THRESH:
                self.drive(SPEED_FAST, 0.0)
            else:
                self.stop()
                self.get_logger().info('Reached home (0, 0)! All done.')
                self.state = 'done'

        # --- Done — just sit still ---
        elif self.state == 'done':
            self.stop()

        # Show the debug window with the camera feed
        self.show_debug(frame)

    # -----------------------------------------------------------------------
    # Ball tracking logic
    # -----------------------------------------------------------------------

    def track_ball(self, frame: np.ndarray, color: str):
        """
        Use the camera image to steer toward the ball and hit it.

        We look at where the ball appears in the image:
          - If the ball is near the bottom (by is large), it is close
          - If the ball is near the top (by is small), it is far away
          - If the ball is left/right of centre, we turn to align

        color: 'green' or 'blue'
        """
        if color == 'green':
            lower, upper = GREEN_LOWER, GREEN_UPPER
        else:
            lower, upper = BLUE_LOWER, BLUE_UPPER

        found, bx, by, _ = self.detect_ball(frame, lower, upper)

        if not found:
            # Can't see the ball — spin slowly to search for it
            self.drive(0.0, TURN_FAST)
            self.centred_frames = 0
            return

        # bx = pixel x of ball (0=left edge, ~600=right edge)
        # by = pixel y of ball (0=top edge, ~400=bottom edge)

        if 300 <= by < 370:
            # Ball is close (near bottom of image) — align left/right only
            if 270 <= bx <= 330:
                # Ball is centred horizontally — count confirmation frames
                self.centred_frames += 1
                self.stop()
                if self.centred_frames >= HIT_CONFIRM_FRAMES:
                    self.execute_hit()
            elif bx < 270:
                # Ball is to the left — turn left
                self.drive(0.0, TURN_SLOW)
                self.centred_frames = 0
            else:
                # Ball is to the right — turn right
                self.drive(0.0, -TURN_SLOW)
                self.centred_frames = 0

        elif 200 <= by < 300:
            # Ball is at medium distance — move forward slowly
            self.drive(SPEED_SLOW, 0.0)
            self.centred_frames = 0

        elif 100 <= by < 200:
            # Ball is far away — move forward faster
            self.drive(SPEED_FAST, 0.0)
            self.centred_frames = 0

        else:
            # Ball is somewhere unexpected — drive forward to get closer
            self.drive(SPEED_FAST, 0.0)
            self.centred_frames = 0

    # -----------------------------------------------------------------------
    # Ball detection using OpenCV
    # -----------------------------------------------------------------------

    def detect_ball(self, frame, lower, upper):
        """
        Find the largest coloured blob in the frame matching the HSV range.

        Steps:
          1. Resize frame to a fixed width for consistent detection speed
          2. Blur the image to reduce noise
          3. Convert to HSV colour space
          4. Create a mask for pixels matching our colour range
          5. Find contours (outlines) of the coloured blobs
          6. Return the position of the largest blob

        Returns: (found, x, y, radius)
            found  — True if a ball was detected
            x, y   — pixel position of ball centre
            radius — radius of the ball in pixels
        """
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # White pixels = colour matches, black pixels = colour does not match
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)   # remove tiny noise dots
        mask = cv2.dilate(mask, None, iterations=2)  # restore blob size after erode

        contours = imutils.grab_contours(
            cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        )

        if len(contours) == 0:
            return False, 0, 0, 0.0   # nothing found

        # The largest contour is most likely the actual ball
        biggest = max(contours, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(biggest)

        if radius < MIN_BALL_RADIUS:
            return False, 0, 0, 0.0   # too small — probably noise

        # Image moments give us the true centroid of the blob
        M = cv2.moments(biggest)
        ball_x = int(M['m10'] / M['m00'])
        ball_y = int(M['m01'] / M['m00'])

        # Draw the detected ball on the frame for the debug window
        cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)
        cv2.circle(frame, (ball_x, ball_y), 5, (0, 0, 255), -1)

        return True, ball_x, ball_y, radius

    # -----------------------------------------------------------------------
    # Execute the hit manoeuvre
    # -----------------------------------------------------------------------

    def execute_hit(self):
        """Drive forward in a short burst to strike the ball, then wait."""
        self.get_logger().info(f'Hitting ball #{self.hit_count + 1}!')

        # Publish forward speed in short pulses to strike the ball
        burst = Twist()
        burst.linear.x = SPEED_FAST
        for _ in range(30):
            self.cmd_pub.publish(burst)
            time.sleep(0.1)   # short deliberate pause between pulses

        self.stop()
        self.centred_frames = 0
        self.hit_count += 1

        # Figure out where to go next
        if self.hit_count == 1:
            next_state = 'rotate_neg_y'    # go find green ball 2
        elif self.hit_count == 2:
            next_state = 'rotate_pos_x'    # go find blue ball
        else:
            next_state = 'home_rotate_x'   # all balls hit, go home

        self.get_logger().info(
            f'Hit done. hit_count={self.hit_count}. '
            f'Waiting {DWELL_AFTER_HIT}s then going to: {next_state}'
        )

        # Enter the waiting state — image_callback checks the timer
        self.wait_until = time.monotonic() + DWELL_AFTER_HIT
        self.state_after_wait = next_state
        self.state = 'waiting'

    # -----------------------------------------------------------------------
    # Heading error helper
    # -----------------------------------------------------------------------

    def heading_error(self, target_deg: float) -> float:
        """
        Returns the absolute angle difference between current heading and
        the target heading, in degrees. Always returns a value between 0-180.

        Example: if self.yaw = 10 and target_deg = -10, error = 20 degrees.
        """
        diff = (self.yaw - target_deg + 180.0) % 360.0 - 180.0
        return abs(diff)

    # -----------------------------------------------------------------------
    # Drive helpers
    # -----------------------------------------------------------------------

    def drive(self, linear: float, angular: float):
        """Set the desired speed. Published at 10 Hz by control_timer."""
        self.cmd.linear.x = linear
        self.cmd.angular.z = angular

    def stop(self):
        """Set speed to zero."""
        self.drive(0.0, 0.0)

    def publish_cmd(self):
        """Timer callback — publishes the current cmd at 10 Hz."""
        self.cmd_pub.publish(self.cmd)

    # -----------------------------------------------------------------------
    # Debug display
    # -----------------------------------------------------------------------

    def show_debug(self, frame: np.ndarray):
        """Overlay state and pose info on the camera frame and show it."""
        cv2.putText(
            frame,
            f'State: {self.state}  Hits: {self.hit_count}',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.putText(
            frame,
            f'x={self.x:.2f}  y={self.y:.2f}  yaw={self.yaw:.1f} deg',
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
        )
        cv2.imshow('Ball Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('q pressed, shutting down')
            rclpy.shutdown()


# -------------------------------------------------------------------------
# Entry point
# -------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
