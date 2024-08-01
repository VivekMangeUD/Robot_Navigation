CISC849-015 #HW2

Contributors:
----------------

Vivek Mange


Code Execute:
-------------------

python3 wanderer.py

python3 reactor.py


Code Flow:
--------------------

Wanderer:
-------------------

<!-- After the execution of the script, By default, The forward flag is True indication the robot to move forward. 
Meanwhile, we continuously subscribe to hazard detection topic. Once we detect a bump, The forward flag is turn false and we send 0 Velocity to the robot.
And the Check_backup limit flag is turned on to monitor the backup limit hazard message. The robot moves backward till backup limit is hit.
Now, The robot has to perform rotation and we have randomize the rotation process. The way we decided how the robot would pick a direction was to randomize either left or right and then we put the threshold of angular velocity in the range [-3.00rad/sec - +3.00rad/sec], further the
time period to send the data is randomized between 5 to 15 seconds. Once the rotation is complete, The forward flag is turned on and the robot moves forward. -->

After the execution of the script, By default, The forward flag is True indicating the robot to move forward. Meanwhile, we continuously subscribe to hazard detection topics. Once we detect a bump, The forward flag turns false and we send 0 Velocity to the robot.And the Check_backup limit flag is turned on to monitor the backup limit hazard message. The robot moves backward till the backup limit is hit. Now, The robot has to perform rotation and we have randomized the rotation process. The way we decided how the robot would pick a direction was to randomize either left or right and then we put the threshold of angular velocity in the range [-3.00rad/sec - +3.00rad/sec], further the the time period to send the data is randomized between 5 to 15 seconds. Once the rotation is complete, The forward flag is turned on and the robot moves forward.



Reactor:
-----------------

For the reactor, we are continuously subscribed to the IR intensity data we monitor if there
is a threat or object in front of each 7 IR sensor. We store this values in a list. 
To avoid the threat we have tested 2 methods as disccused below:
1. We split the 7 Ir readings in 2 zones, Left and Right. The sensors 5-7 are zone Left and sensors 0-4 are zone Right. We check the index of the maximum value of IR intesity, And if the maximum values is in zone Left, we send the robot to move left and opposite for the zone Right. The velocities and time period of rotation are choosen at random between (0, 3) and (-3,0) rads/s for rotations in respective zones and (5,15) secs as timeperiod.

2. In this method, Instead of spliting the range in 2 zones, we update the angular velocity sent to the robot and update it based on the closet threat. for eg, If the threat is very close to far end Ir sensors at left or right, we send low angular velocity as it would be sufficent to avoid the obsatcle. As the threat gets more and more close to the center of the robot, we send higher anglular velocity to avoid the threat.
