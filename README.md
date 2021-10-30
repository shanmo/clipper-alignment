# CLIPPER object-based map alignment 

ROS package for inter-robot frame alignment based on jointly observed objects.
The node "aligner" uses the CLIPPER algorithm to establish correspondence between the objects.

Demo video with 3 robots: https://youtu.be/v5jIy5QEPiY

## Expected topics
The node "aligner" will subscribe to topics named as
"robot1/landmarks"
"robot1/pose"
"robot2/landmarks"
"robot2/pose"
in the case of 2 robots, for example. 

The topic "landmarks" is expected to be of the sensor_msgs::PointCloud2 type, which is the object/landmark map. 
The x-y-z position of each point indicates the location of the object in the robot's (local) map.
The topic "pose" is of geometry_msgs::PoseStamped type.
This is the pose of the robot from start time until now. 

The node will publish the topics
"/robot1/viz/path"
"/robot1/viz/frame"
"/robot1/landmarks_aligned"
"/robot2/viz/path"
"/robot2/viz/frame"
"/robot2/landmarks_aligned"
"/robot2/viz/correspondences"
in the case of 2 robots, for example. 

The topic "landmarks_aligned" is the aligned object map of robot i to robot 1 frame. 
We always set robot 1 as the reference frame.
The topic "correspondences" show the associations found by CLIPPER between robot i and robot 1 objects.

## Launch file and parameters
See "aligner.launch" in the launch folder.
See "aligner_demo.launc" for a demo on UCSD bags.

Number of robots must be provided to the node using the (private) ros param "num_robots". Otherwise, it will take the default value of 1.

The thresholds used by the CLIPPER algorithm can also be set using the parameters:
- "clipper_sigma": sigma for the Guassian weighting
- "clipper_epsilon": maximum error to consider two associations consistent
- "clipper_mindist": minimum distance between chosen objects in same map


## Conversion from MarkerArray to PointCloud2
If object maps are broadcasted in the MarkerArray format, then the node "marker2pt2" can be used to convert them to PointCloud2.
For example, see "load_orcvio_bags.launch".
 