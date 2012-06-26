/*
Collision Avoidance Node

This node controls the collision avoidance algorithm--an implementation of reactive inverse PN.
*/

//standard C++ headers
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <queue>
#include <map>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include <boost/lexical_cast.hpp>

//ROS headers
#include "ros/ros.h"
#include "sim/TelemetryUpdate.h"
#include "sim/GoToWaypoint.h"
#include "sim/RequestWaypointInfo.h"
#include "sim/standardDefs.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

//our headers
#include "sim/planeObject.h"
#include "sim/standardFuncs.h"
#include "sim/ripna.h"


#define WEST_MOST_LONGITUDE -85.490356
#define NORTH_MOST_LATITUDE 32.606573


#define METERS_TO_LATITUDE (1.0/111200.0)

#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)

//publisher is global so callbacks can access it
ros::Publisher marker_pub;

struct waypoint
{
	double latitude;
	double longitude;
	double altitude;
};

struct waypoint origin;

double distance(struct waypoint first, struct waypoint second)
{
	//difference in latitudes in radians
	double lat1 = first.latitude*DEGREES_TO_RADIANS;
	double lat2 = second.latitude*DEGREES_TO_RADIANS;
	double long1 = first.longitude*DEGREES_TO_RADIANS;
	double long2 = second.longitude*DEGREES_TO_RADIANS;
	
	double deltaLat = lat2 - lat1;
	double deltaLong = long2 - long1;
	
	//haversine crazy math, should probably be verified further beyond basic testing
	//calculate distance from current position to destination
	double a = pow(sin(deltaLat / 2.0), 2);
	a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
	a = 2.0 * asin(sqrt(a));
	
	return EARTH_RADIUS * a;
}


/* ROS service clients for calling services from the coordinator */
ros::ServiceClient goToWaypointClient;
ros::ServiceClient requestWaypointInfoClient;

int count, numPlanes, currentPlaneID; /* keeps count of the number of goToWaypoint services requested, 
the number of planes in the airspace, and the current planes ID */



std::map<int, sim::PlaneObject> planes; /* map of planes in the airspace.  The key is the plane id of the aircraft */

/* This function is run every time new telemetry information from any plane is recieved.  With the new telemetry update, 
information about the plane is updated, including bearing, speed, current location, etc.  Additionally, we check to see
if the UAV has reached it's current destination, and, if so, update the destination of the UAV.
After updating, the calculateForces function is called to find a the new force acting on the UAV; from this new force,
a next waypoint is found and forwarded to the coordinator. */
void telemetryCallback(const sim::TelemetryUpdate::ConstPtr &msg);


int main(int argc, char **argv)
{	
	//standard ROS startup
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	/*subscribe to telemetry outputs and create clients for the 
	goToWaypoint and requestWaypointInfo services*/
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	goToWaypointClient = n.serviceClient<sim::GoToWaypoint>("go_to_waypoint");
	requestWaypointInfoClient = n.serviceClient<sim::RequestWaypointInfo>("request_waypoint_info");
	
	//publish basic shapes to plane positions
  	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	//initialize counting
	count = 0;	
	origin.latitude=NORTH_MOST_LATITUDE;
	origin.longitude=WEST_MOST_LONGITUDE;
	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}

void telemetryCallback(const sim::TelemetryUpdate::ConstPtr &msg)
{	
	int planeID = msg->planeID;

	/* Instantiate services for use later, and get planeID*/
	sim::GoToWaypoint goToWaypointSrv;
	sim::GoToWaypoint goToWaypointSrv2;
	sim::RequestWaypointInfo requestWaypointInfoSrv;
	

	/* Request this plane's current normal destination */
	requestWaypointInfoSrv.request.planeID = planeID;
	requestWaypointInfoSrv.request.isAvoidanceWaypoint = false;
	requestWaypointInfoSrv.request.positionInQueue = 0;

	if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
		ROS_ERROR("Did not receive a response from the coordinator");
		return;
	}

	/* If the plane has reached its current destination, move onto the 
	next destination waypoint. This does not set the destination
	of the plane object in the map "planes." */
	if (findDistance(msg->currentLatitude, msg->currentLongitude, 
					requestWaypointInfoSrv.response.latitude, 
					requestWaypointInfoSrv.response.longitude) < COLLISION_THRESHOLD){

		/* request next normal destination */
		requestWaypointInfoSrv.request.positionInQueue = 1;

		if (!requestWaypointInfoClient.call(requestWaypointInfoSrv)){
			ROS_ERROR("Did not recieve a response from the coordinator");
			return;
		}
	}


	/* If the plane is not in our map of planes and has destination
	waypoints, then add it as a new plane to our map of planes. 
	
	Else if the plane is not in our map of planes and does not
	have waypoints, return and do nothing more. */
	if (planes.find(planeID) == planes.end() && msg->currentWaypointIndex != -1){ 
		/* This is a new plane, so create a new planeObject and 
		give it the appropriate information */
		sim::PlaneObject newPlane(MPS_SPEED, *msg); 
		planes[planeID] = newPlane; /* put the new plane into the map */

		/* Update the destination of the PlaneObject with the value 
		found with the requestWaypointInfoSrv call */
		sim::waypoint newDest; 
		newDest.latitude = requestWaypointInfoSrv.response.latitude;
		newDest.longitude = requestWaypointInfoSrv.response.longitude;
		newDest.altitude = requestWaypointInfoSrv.response.altitude;

		planes[planeID].setDestination(newDest);

		numPlanes += 1;
		return;
	}
	else if (planes.find(planeID) == planes.end()) 
		/* New plane without waypoint set */
		return; 
	

	/* Note: The requestWaypointInfo service returns a waypoint of 
	-1000, -1000 when the UAV cannot retrieve a destination from queue.

	If the plane has no waypoint to go to, put it far from all others.

	Or, if the plane does have a waypoint to go to, update the plane 
	with new position and destination received from requestWaypointInfoSrv
	response*/
	if (requestWaypointInfoSrv.response.latitude == -1000){ /* plane has no waypoints to go to */
		/* Remove in real flights*/
		planes[planeID].setCurrentLoc(-1000,-1000,400);
		/* update the time of last update for this plane to acknowledge 
		it is still in the air */
		planes[planeID].updateTime(); 
		return; 
	}
	else{
		planes[planeID].update(*msg); /* update plane with new position */

		sim::waypoint newDest;

		newDest.latitude = requestWaypointInfoSrv.response.latitude;
		newDest.longitude = requestWaypointInfoSrv.response.longitude;
		newDest.altitude = requestWaypointInfoSrv.response.altitude;

		planes[planeID].setDestination(newDest); /* update plane destination */
	}


	/* Each plane calls the collision avoidance algorithm */

	if (planeID != numPlanes - 1) {
		return;
	}

	sim::waypoint newWaypoint;

	for (int n = 0; n < numPlanes; n++) {
		/* This line of code calls the collision avoidance algorithm 
		and determines if there should be collision avoidance 
		maneuvers taken. Returns a waypoint for the plane to go to.  
		*/	

		currentPlaneID = n;
		
		newWaypoint = findNewWaypoint(planes[currentPlaneID], planes);
	
		if ((requestWaypointInfoSrv.response.longitude == newWaypoint.longitude) 
			&& (requestWaypointInfoSrv.response.latitude == newWaypoint.latitude)) {
		return;
		}	

		/* Fill in goToWaypointSrv request with new waypoint information*/
		goToWaypointSrv.request.planeID = currentPlaneID;
		goToWaypointSrv.request.latitude = newWaypoint.latitude;
		goToWaypointSrv.request.longitude = newWaypoint.longitude;
		goToWaypointSrv.request.altitude = newWaypoint.altitude;
		goToWaypointSrv.request.isAvoidanceManeuver = true; 
		goToWaypointSrv.request.isNewQueue = true;

		if (goToWaypointClient.call(goToWaypointSrv)) {
			count++;
			ROS_INFO("Received response from service request %d", (count-1));
		}
		else {
			ROS_ERROR("Did not receive response");
		}

	


		//request waypoint info to publish a square at each plane's next COLLISION AVOIDANCE waypoint location.
		/*sim::RequestWaypointInfo srv2;
		srv2.request.planeID = msg->planeID;	
		srv2.request.isAvoidanceWaypoint = true;
		srv2.request.positionInQueue = 0;//msg->currentWaypointIndex; */
	
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		char buffer [5];
		sprintf(buffer, "%d",planeID);

		//set up verticies of triangle
		struct waypoint northsouthpointWP2;
		northsouthpointWP2.latitude=newWaypoint.latitude;//srv->currentLatitude;
		northsouthpointWP2.longitude=WEST_MOST_LONGITUDE;
		struct waypoint eastwestpointWP2;
		eastwestpointWP2.latitude=NORTH_MOST_LATITUDE;
		eastwestpointWP2.longitude=newWaypoint.longitude;//srv->currentLongitude;

		std::stringstream sstm4;
		sstm4 << "CA" << buffer;


		transform.setOrigin( tf::Vector3(distance(origin,eastwestpointWP2), -distance(origin,northsouthpointWP2),0));
	  	transform.setRotation( tf::Quaternion(0, 0, 0) );
	  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", sstm4.str()));

		//set shape
		uint32_t shape = visualization_msgs::Marker::SPHERE;

		visualization_msgs::Marker marker4;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		std::stringstream sstm5;
		sstm5 << "/CA" << buffer;
		marker4.header.frame_id = sstm5.str();

		marker4.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker4.ns = "basic_shapes";
		marker4.id = planeID+3000;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker4.type = shape;

		// Set the marker action.  Options are ADD and DELETE
		marker4.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker4.pose.position.x = 0;
		marker4.pose.position.y = 0;
		marker4.pose.position.z = 0;
		marker4.pose.orientation.x = 0.0;
		marker4.pose.orientation.y = 0.0;
		marker4.pose.orientation.z = 0.0;
		marker4.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker4.scale.x = 12.0;
		marker4.scale.y = 12.0;
		marker4.scale.z = 12.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker4.color.r = 1.0f;
		marker4.color.g = 0.0f;
		marker4.color.b = 0.0f;
		marker4.color.a = 0.7f;

		marker4.lifetime = ros::Duration();

		// Publish the marker
		marker_pub.publish(marker4);	

	}
	
}




