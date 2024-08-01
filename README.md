# README

#### Contributor
- Vivek Mange

---

#### Robot

**Turtlebot Waffle** with custom Lidar and depth camera mount. Enhanced with RPI, Rplidar, and Realsense camera for better performance.


---
#### Files

##### 1. Laser Scan
To execute: 

**python3 splinter_wall_following.py**

Algorithm:

- Robot follows the wall on the left side.
- Laser scan divided into front (0-100, 900-1000) and left (200-400) regions.
- Moves forward with constant velocity, monitors front ranges, and performs emergency stop if needed.
- Adjusts rotation and forward movement to maintain a 0.4m distance from the wall.

---

#### 2. Reactor and Wanderer
To execute:

**python3 wanderer.py**
**python3 reactor.py**

**i. Wanderer:**

- Robot moves forward by default.
- Stops and moves backward when a bump is detected.
- Randomizes rotation direction and duration before resuming forward movement.

**ii. Reactor:**

- Monitors IR intensity data to detect threats.
- Uses two methods to avoid threats:
- Splits IR readings into left and right zones.
- Updates angular velocity based on the closest threat.

---

#### 3. Minesweeper

Task: 

Destroy three mines (two green, one blue) and return to base.


Algorithm:

- Uses Realsense depth camera and OpenCV for color detection.
- Finds and pushes green mines first, then the blue mine.
- Returns to base after hitting all mines.

---