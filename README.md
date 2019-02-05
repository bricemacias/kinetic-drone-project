# Hand Control Beebop Drone with Kinect project

In order to make use of this code, you will need a beebop drone, a Microsoft Kinect, ROS, and Rviz installed on your computer, with some libraries to import in order to communicate with both the drone and the Kinect.

You will also need a coloured glove in order for the kinect to recognize your hand from the environment. In our case, we used a green glove, and the code is made in order to detect that colour (which you can change if you want).

Launch file initializes the environment on the computer in order to prepare it for receiving ROS messages from beebop and Kinect cameras, while launching Rviz, which will help us visualize the data from the cameras and the applied PCA on the glove which will extract the 3 axis from the hand on real time.

Python file is the main script. It will prepare the computer to receive Kinect and beebop messages, and treat information coming from both in order to make the beebop fly acording to the hand distance from the Kinect in terms of height, and according to the angles of rotation in terms of inclination. In order to do so, we applied a PCA on the detected hand surface, in order to get 3 main axis from the flat hand positionned in front of the Kinect, at minimum distance (in order to allow the Kinect to detect it). Then we compare both the height of the drone and the distance of the hand from the Kinect on one side, and the angles of the hand axis with the drone inclination on the other, in order to make the drone follow the hand movements with a scaling factor. We also added security limits (hands desapears, axes not readable, hand out of range etc...)

The code may not work for you, because it depends on a lot of factors, beginning with environmental issues like light, or libraries issues (Some of them may have been updated since and the code could be therefore outdated). However, feel free to try an understand it if you wish to make your own. It will give you good intuition in order to know what you should do in order to acheive it. 

Have Fun !


