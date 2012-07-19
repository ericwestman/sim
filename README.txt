This repository contains code developed as part of the 2012 REU on Smart UAVs at Auburn University.

Its main feature is an original collision avoidance algorithm to be used in multi-UAV situations. The
algorithm ("IPN++") was created by David Fish and Eric Westman, and was based on research by Dr. George
and Dr. Ghose.

IPN++ uses Inverse Proportional Navigation to guide UAVs safely through a 2D airspace. It has been tested
in a variety of stressful simulated situations and has been proven effective in both sparse and dense
airspaces.

For testing and development, we use the Robotics Operating System (ROS) framework and a custom simulation
environment called The UAV ATTRACT project (written by James Holt, it is available at
github.com/holtjma/AU-UAV-ROS). Before using this code, it is highly recommend that you visit Holt's
repository to learn how our algorithm interacts with the ROS environment.

---

The IPN++ algorithm is written in C++ and is located in two files: collisionAvoidance.cpp and ripna.cpp. The
collisionAvoidance.cpp file is used as a Node in ROS. The CollisionAvoidance Node basically acts as a wrapper
around IPN++. The CollisionAvoidance Node sends location information for active UAVs to IPN++ and receives a
new waypoint for each UAV. The majority of IPN++ is located in ripna.cpp, which is called in
collisionAvoidance.cpp.

This version of IPN++ is optimized for a simulator. Unlike the "real" repository
(github.com/circlelabs/real), which is optimized for the real world, this version waits until it has received
updates from all UAVs before creating waypoints. This change ensures that IPN++ will use the newest update
information for all UAVs, preventing the creation of outdated and irrelevant waypoints.

UAVs are represented as planeObject objects, which are defined in planeObject.cpp. IPN++ relies on a variety
of mathematical functions, which are located in vmath.cpp (vector math), planeObject.cpp, and
standardFuncs.cpp. All source code is located in the src directory.

For questions about usage, documentation, and our results, please visit our project website at
sites.google.com/site/auburnUAVREU2012team5. There, you will find contact information for Eric and me, our
papers, information about the 2012 REU on Smart UAVs, and other useful information.