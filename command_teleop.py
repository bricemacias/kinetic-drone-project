#!/usr/bin/python
# coding: utf-8

import rospy
from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import colorsys
import struct 
import numpy as np
import time
from math import sqrt

#################################################
###### Fonction pour publier le pointcloud ######
#################################################

def publish_points(points_pub, points_list):
    pcl_msg = PointCloud2()
    pcl_msg.header.frame_id = "camera_rgb_optical_frame"
    pcl_msg.header.stamp = rospy.Time.now()
    pcl_msg.width = len(points_list)
    pcl_msg.height  = 1
    pcl_msg.fields.append(PointField(
        name = "x",offset = 0,
        datatype = PointField.FLOAT32,
        count = 1 ))
    pcl_msg.fields.append(PointField(
        name = "y",offset = 4,
        datatype = PointField.FLOAT32,count = 1 ))
    pcl_msg.fields.append(PointField(
        name = "z",offset = 8,
        datatype = PointField.FLOAT32,count = 1 ))
    pcl_msg.point_step = 12
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width * pcl_msg.height
    buffer = []
    for x,y,z in points_list:
        buffer.append(struct.pack('fff',x, y, z))

    pcl_msg.data = "".join(buffer)
    points_pub.publish(pcl_msg)

########################################################
###### Fonction pour créer les vecteurs pour Rviz ######
########################################################

def markerVector(id,vector,position, color):
    marker = Marker ()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "arrow";
    marker.id = id;
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.scale.x=0.01
    marker.scale.y=0.02
    marker.scale.z=0
    marker.color.a= 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    (start,end)=(Point(),Point())
 
    start.x = position[0]
    start.y = position[1]
    start.z = position[2]
    end.x=start.x+0.1 * vector[0]
    end.y=start.y+0.1 *vector[1]
    end.z=start.z+0.1 * vector[2]
 
    marker.points.append(start)
    marker.points.append(end)
    return marker

####################################################################
###### Fonctions utilisées pour isoler la main du pointcloud #######
####################################################################

def unpack_rgb(rgb_float, big_endian=False):
    # Get the bytes of the float rgb_float
    fmt = "f"
    if(big_endian):
        fmt = "!" + fmt
    packed = struct.pack(fmt , rgb_float)
    integers = [ord(c) for c in packed]
    return integers[0], integers[1], integers[2]

def rgb2hsv(r,g,b):
        (h1,s1,v1) = colorsys.rgb_to_hsv(r*(1./255),g*(1./255),b*(1./255))
	(h,s,v) = (int(h1*180),int(s1*255),int(v1*255))
	return (h, s, v)
 
def inRange(h,s,v, hsvmin, hsvmax):
    return (h >= hsvmin[0] and h <= hsvmax[0]
        and s >= hsvmin[1] and s <= hsvmax[1]
        and v >= hsvmin[2] and v <= hsvmax[2])

    
###################################################
######### Fonction de pilotage du drone ###########
###################################################

def angleToData(angle):
        return (angle - 10.0)/60.0

def fly_drone(command_pub, alphaAngle, betaAngle, gammaAngle, zValue, hand, flying_bool):
    twist = Twist()
    i = 0
    while (i < 3):
        if(hand == False):
            twist.linear.x = twist.linear.y = twist.linear.z = twist.linear.z = twist.angular.x = twist.angular.y = twist.angular.z = 0
            command_pub.publish(twist)
            time.sleep(4)
            i += 1
        elif(i == 2):
            rospy.Publisher('land', Empty, queue_size=1).publish(Empty())
            print("on envoie la commande d'aterissage")
            flying_bool = False
            i = 3
        elif(hand == True and i <= 1):
            while(hand == True):
                if (abs(alphaAngle) > 70 or abs(betaAngle) > 70 or abs(gammaAngle) > 70 or (-10 <= alphaAngle <= 10 and -10 <= betaAngle <= 10 and-10 <= gammaAngle <= 10)):
                    twist.linear.x = twist.linear.y = twist.linear.z = twist.linear.z = twist.angular.x = twist.angular.y = twist.angular.z = 0

                elif(10 < abs(alphaAngle) <= 70):
                    twist.linear.x = angleToData(alphaAngle)
                    
                elif(10 < abs(betaAngle) <= 70):
                    twist.linear.y = -angleToData(betaAngle)

                elif(10 < abs(gammaAngle) <= 70):
                    twist.angular.z = -angleToData(gammaAngle)

                #rajouter un elif pour z

                else:
                    twist = Twist()
                   
                command_pub.publish(twist)
            
#####################################################
#####################################################
################ Fonction centrale ##################
#####################################################
#####################################################

def pcl_cb(points_msg, hsvmin, hsvmax, points_pub, markers_pub, data_pub, debug=False):

    gen = pc2.read_points(points_msg, skip_nans=True)
    
    
    filtered_points = []
    x_vector = []
    y_vector = []
    z_vector = []

    for x,y,z,rgb in gen:
        (r,g,b) = unpack_rgb(rgb)
	(h,s,v) = rgb2hsv(r,g,b)
	if(inRange(h,s,v, hsvmin, hsvmax) and z <= 1.0 and -0.2 <= x <= 0.2 and -0.2 <= y <= 0.2):
		#print ((h,s,v))
		#print ((x,y,z))
		x_vector.append(x)
		y_vector.append(y)
		z_vector.append(z)
		filtered_points.append((x,y,z))
                    
        publish_points(points_pub, filtered_points)

#######################################
############### PCA ###################
#######################################

    if(len(filtered_points) == 0):
        if(debug):
            print("I did not keep any points")
            #on utilise hand_presence pour l'utiliser dans la fonction
            #de pilotage du drone
            hand_presence = False
	return 
    else:
        hand_presence = True
	
    mean_x = np.mean(x_vector)
    mean_y = np.mean(y_vector)
    mean_z = np.mean(z_vector)
    
    x_centered = x_vector - mean_x
    y_centered = y_vector - mean_y
    z_centered = z_vector - mean_z
    
    mean_vector = np.array([[mean_x],[mean_y],[mean_z]])
    mean_values = (mean_x, mean_y, mean_z)
    
    cov_mat = np.cov([x_centered,y_centered,z_centered])
	
    # eigenvectors and eigenvalues for the from the covariance matrix
    try:
    	eig_val_cov, eig_vec_cov = np.linalg.eig(cov_mat)
        hand_presence = True
    except np.linalg.LinAlgError:
        print "No points detected"
        hand_presence = False
    	return

    # Order the eigen values by increasing values
    permutation = np.argsort(eig_val_cov)

    # We keep the two largest
    hand_length_eig_idx = permutation[2]
    hand_width_eig_idx = permutation[1]

    hand_length_eigvec = eig_vec_cov[:, hand_length_eig_idx]
    hand_width_eigvec = eig_vec_cov[:, hand_width_eig_idx]

    hand_length_eigval = eig_val_cov[hand_length_eig_idx]
    hand_width_eigval = eig_val_cov[hand_width_eig_idx]
    
    # flip les vecteurs pour qu'ils pointent toujours "dans la même direction"
    # hand_length : vers le dessus de la kinect
    if(hand_length_eigvec[1] >= 0):
    	hand_length_eigvec *= -1
    if(hand_width_eigvec[0] <= 0):
        hand_width_eigvec *= -1


    eigen_vectors = (hand_length_eigvec, hand_width_eigvec)
    eigen_values = (hand_length_eigval, hand_width_eigval)

####################################### 
############### Angles ################
#######################################

    def vectorNorm(a):
        return sqrt(a[0]**2 + a[1]**2)

   

    #angle alpha sur le plan -y et z

    yRefAlpha = np.array([-1.0, 0.0])
    alphaVec = np.array([hand_length_eigvec[1], hand_length_eigvec[2]])
    alpha = np.arccos(np.inner(yRefAlpha, alphaVec)/(vectorNorm(yRefAlpha)*vectorNorm(alphaVec)))
    if(hand_length_eigvec[2] >= 0):
        alpha *= -1
    
    alphaDeg = np.rad2deg(alpha)
    
    #angle beta sur le plan x et z

    xRefBeta = np.array([1.0, 0.0])
    betaVec = np.array([hand_width_eigvec[0], hand_width_eigvec[2]])
    beta = np.arccos(np.inner(xRefBeta, betaVec)/(vectorNorm(xRefBeta)*vectorNorm(betaVec)))
    if(hand_width_eigvec[2] <= 0):
        beta *= -1
    
    betaDeg = np.rad2deg(beta)


    #angle gamma sur le plan x et -y
    
    yRefGamma = np.array([0.0, -1.0])
    gammaVec = np.array([hand_length_eigvec[0], hand_length_eigvec[1]])
    gamma = np.arccos(np.inner(yRefGamma, gammaVec)/(vectorNorm(yRefGamma)*vectorNorm(gammaVec)))
    if(hand_length_eigvec[0] >= 0):
        gamma *= -1
    
    gammaDeg = np.rad2deg(gamma)



    print(alphaDeg, betaDeg, gammaDeg, hand_length_eigvec, hand_width_eigvec, hand_length_eig_idx, hand_width_eig_idx, hand_length_eigval, hand_width_eigval )

#####################################################
############## Commande de décollage ################
#####################################################
    
    if(flying == False and -10 <= alphaDeg <= 10 and -10 <= betaDeg <= 10 and-10 <= gammaDeg <= 10):
        rospy.Publisher('takeoff', Empty, queue_size=1).publish(Empty())
        flying = True
        print("commande de decollage sent")
        fly_drone(data_pub, alphaDeg, betaDeg, gammaDeg, mean_z, hand_presence,flying) 

                    
###################################################
########## Générateur d'axes pour Rviz ############
###################################################

    id=1

    markers_array = MarkerArray()
    markers_array.markers.append(markerVector(1, hand_length_eigvec, mean_values, (228./255., 48./255., 59./255.)))
    markers_array.markers.append(markerVector(2, hand_width_eigvec, mean_values, (5./255., 226./255, 0./255)))
    markers_pub.publish(markers_array)


####################################################
####### Publishers, Subscribers and Values #########
####################################################


hsvmin = (30, 50, 10)
hsvmax = (60, 150, 255)

rospy.init_node("filter_hand_pointcloud")

pcl_pub = rospy.Publisher("points", PointCloud2, queue_size=1)
markers_pub = rospy.Publisher ("visualization_markers", MarkerArray, queue_size=1)
data_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

xyzrgb_sub = rospy.Subscriber('camera/depth_registered/points', PointCloud2, lambda msg: pcl_cb(msg, hsvmin, hsvmax, pcl_pub, markers_pub, data_pub, True))


rospy.spin()
