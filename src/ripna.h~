/*
RIPNA


This is the header file for RIPNA.cpp, which implements the functions declared here. This file and its comments pertain to the usage of the functions contained in this file.
*/

#ifndef COLLISION_AVOIDANCE_ALGORITHM_H
#define COLLISION_AVOIDANCE_ALGORITHM_H

#include <map>
#include <tuple>

#include "AU_UAV_ROS/planeObject.h"
#include "AU_UAV_ROS/vmath.h"

namespace AU_UAV_ROS{
	
	/*This function is called in collisionAvoidance.cpp and utilizes the other 
	functions outlined in this file to calculate a collision avoidance waypoint 
	for the plane to travel to. If no collision avoidance or maneuvering is 
	necessary, this functions returns the current destination waypoint */
	AU_UAV_ROS::waypoint findNewWaypoint(PlaneObject &plane1, std::map<int, PlaneObject> &planes);
	
	/*This function receives the current plane and a map of all of the planes 
	in the airspace, and returns the ID of the plane which is the most imminent 
	threat to the current plane. */
	std::tuple<int, double> findGreatestThreat(PlaneObject &plane1, std::map<int, PlaneObject> &planes);
	
	/* This function takes the current plane and its greatest threat and 
	returns a bool which indicates whether the plane should turn right or left */
	bool shouldTurnRight(PlaneObject &plane1, PlaneObject &plane2);
	
	/* This function calculates the desired turning radius based on the ZEM*/
	double calculateTurningRadius(double ZEM);
	
	/* This function takes the current plane, its calculated turning radius, 
	and which direction it should turn in order to find the new collision 
	avoidance waypoint for the plane to go to.*/
	AU_UAV_ROS::waypoint calculateWaypoint(PlaneObject &plane1, double turningRadius, bool turnRight);

	/*This function calculates the next waypoint for the plane based on its 
	distance from its current waypoint and its bearing. */
	AU_UAV_ROS::waypoint takeDubinsPath(PlaneObject &plane, AU_UAV_ROS::waypoint &wp);
	
	/* This function takes a plane, its turning radius, and the direction to turn 
	and returns the center of the circle of its turning radius. */
	AU_UAV_ROS::coordinate calculateCircleCenter(PlaneObject &plane, double turnRadius, bool turnRight);
};


#endif
