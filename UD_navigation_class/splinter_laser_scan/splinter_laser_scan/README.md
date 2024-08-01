## To excute the code Run the below command
	python3 splinter_wall_following.py

## Algorithm for the task:
We are following the wall using Left side.
min_dist = 0.4

1. The Robot laserscan is divide in 2 regions 
	1. Front [Range values between 0-100 and 900-1000]
	2. Left [Range values : 200-400]

2. At initial condition, We have the forward flag and check_forward flag as True and the robot moves forward with constnat velocity of 0.1m/s. Also There is a Red Marker, Which indicated the object of reference at the Left side of the robot. defined at the range value 250 inside lidar data.

3. While moving forward, we monitor the Front ranges values and if the minimum value in the list is
below the min_distance values, we stop the robot and Turn the Rotation flag on. There is an emergency stop if the min value from the overall laser data is less than 0.1m.

4. While performing the rotation, we check the range value at 90 degree from 0 lidar facing i.e the range value Marker and perform rotation until the Marker is in between the min_dist +- the Threshold of 0.05m.

5. After Rotation, we change the state to follow the wall: where we keep on moving forward, and the distance between the wall and the robot are montiored by below checklist:

6. Conditions: We check the How Far the marker is from the wall and perform the below mentioned tasks:
	a. If the distance from the wall is within min_distance +-0.05 threshold, we just move forward.
	b. If the robot is moving closer to the wall: i.e the Marker is less than min_distance, we move the robot forward and + some angular velocity in clockwise to result moving away from the wall.
	c. If we move away from the wall, we send a -ve angular veliocity with forward movement to come closer to the wall and maintain the distance.
	d. We have also added some far cases: When the robot is moving away from the wall over certain range, we increase the clockwise rotation to come back to the a bit faster, This condition, helps during wall lost or sudden change in wall direction. 


