# Homework4

After cloning the repository, build the packages by doing:

     $ colcon build

Then, use the source command:

    $ source install/setup.bash

To launch Gazebo and spawn the robot write the following command in a terminal:

    $ ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py

After doing this, remember to press the play button on Gazebo.

To explore the goals as requested in the points 2 and 3, in a second terminal write the following command:

    $ ros2 launch rl_fra2mo_description fra2mo_explore.launch.py

If you want to test the point 4 instead (to test both the navigation and detection skills of the robot), you have to launch in the second terminal the new launch file we created as follows:

    $ ros2 launch rl_fra2mo_description vision_based_navigation.launch.py

In a third terminal write:

    $ ros2 run rl_fra2mo_description follow_waypoints.py

If you want to monitor the robot's movement in RViz, particularly the mapping with the specific configuration required by the task, you can use the following command:

    $ ros2 launch rl_fra2mo_description display_fra2mo.launch.py

# Note: Depending on the point of the homework you want to test, select the corresponding number from the terminal.

If you want to view the image captured by the camera, execute the following command in another terminal:

     $ ros2 run rqt_image_view rqt_image_view

To see the real image, select the /videocamera topic, to see the image with the detection of the ArUco tag instead, select the aruco_single/result topic.

You can check the behaviour of the robot by visiting the following link:

https://youtu.be/g0aNKmdAj38?si=w9UMfGCH_CoLTM0G

The videos have been sped up to allow for faster and easier review.
