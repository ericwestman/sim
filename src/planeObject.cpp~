/* Implementation of planeObject.h

*/

#include "ros/ros.h"
#include "AU_UAV_ROS/planeObject.h"
#include "AU_UAV_ROS/SimulatedPlane.h"

#include <math.h>
#include "AU_UAV_ROS/standardFuncs.h" /* for PI, EARTH_RADIUS in meters */

/* Implementation of the default constructor: Member variables are set to zero */
AU_UAV_ROS::PlaneObject::PlaneObject(void) {
	this->id = 0;
	this->currentLoc.altitude = 0.0;
	this->currentLoc.latitude = 0.0;
	this->currentLoc.longitude = 0.0;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = 0.0;
	this->currentBearing = 0.0;

	this->speed = 0.0;
	this->destination.latitude = 0.0;
	this->destination.longitude = 0.0;
	this->destination.altitude = 0.0;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = 0.0;
}

/* Explicit value constructor using TelemetryUpdate */
AU_UAV_ROS::PlaneObject::PlaneObject(double cRadius, const AU_UAV_ROS::TelemetryUpdate &msg) {
	this->id = msg.planeID;
	this->currentLoc.altitude = msg.currentAltitude;
	this->currentLoc.latitude = msg.currentLatitude;
	this->currentLoc.longitude = msg.currentLongitude;
	this->previousLoc.altitude = 0.0;
	this->previousLoc.latitude = 0.0;
	this->previousLoc.longitude = 0.0;
	this->targetBearing = msg.targetBearing;
	this->currentBearing = 0.0;

	this->speed = msg.groundSpeed;
	this->destination.latitude = msg.destLatitude;
	this->destination.longitude = msg.destLongitude;
	this->destination.altitude = msg.destAltitude;
	this->lastUpdateTime = ros::Time::now().toSec();
	this->collisionRadius = cRadius;
}

/* mutator functions to update member variables */
void AU_UAV_ROS::PlaneObject::setID(int id){
	this->id = id;
}

void AU_UAV_ROS::PlaneObject::setPreviousLoc(double lat, double lon, double alt) {
	this->previousLoc.latitude = lat;
	this->previousLoc.longitude = lon;
	this->previousLoc.altitude = alt;
}

void AU_UAV_ROS::PlaneObject::setCurrentLoc(double lat, double lon, double alt) {
	this->currentLoc.latitude = lat;
	this->currentLoc.longitude = lon;
	this->currentLoc.altitude = alt;
}

void AU_UAV_ROS::PlaneObject::setTargetBearing(double tBearing) {
	this->targetBearing = tBearing;
}

void AU_UAV_ROS::PlaneObject::setCurrentBearing(double cBearing) {
	this->currentBearing = cBearing;
}

void AU_UAV_ROS::PlaneObject::setSpeed(double speed) {
	this->speed = speed;
}

void AU_UAV_ROS::PlaneObject::setDestination(const AU_UAV_ROS::waypoint &destination) {
	this->destination = destination;
}

void AU_UAV_ROS::PlaneObject::updateTime(void) {
	this->lastUpdateTime = ros::Time::now().toSec();
}

void AU_UAV_ROS::PlaneObject::update(const AU_UAV_ROS::TelemetryUpdate &msg) {

	this->setTargetBearing(msg.targetBearing);

	//Calculate actual Cardinal Bearing
	double numerator = (this->currentLoc.latitude - this->previousLoc.latitude);
	double denominator = (this->currentLoc.longitude - this->previousLoc.longitude);
	double angle = atan2(numerator*DELTA_LAT_TO_METERS,denominator*DELTA_LON_TO_METERS)*180/PI;
	this->setCurrentBearing(toCardinal(angle));

	this->setPreviousLoc(this->currentLoc.latitude, this->currentLoc.longitude, this->currentLoc.altitude);
	this->setCurrentLoc(msg.currentLatitude, msg.currentLongitude, msg.currentAltitude);
	this->setSpeed(msg.groundSpeed);
	this->updateTime();
}

/* accessor functions */
int AU_UAV_ROS::PlaneObject::getID(void) const {
	return this->id;
}

AU_UAV_ROS::coordinate AU_UAV_ROS::PlaneObject::getPreviousLoc(void) const {
	return this->previousLoc;
}

AU_UAV_ROS::coordinate AU_UAV_ROS::PlaneObject::getCurrentLoc(void) const {
	return this->currentLoc;
}

double AU_UAV_ROS::PlaneObject::getTargetBearing(void) const {
	return this->targetBearing;
}

double AU_UAV_ROS::PlaneObject::getCurrentBearing(void) const {
	return this->currentBearing;
}
	
double AU_UAV_ROS::PlaneObject::getSpeed(void) const {
	return this->speed;
}

double AU_UAV_ROS::PlaneObject::getLastUpdateTime(void) const {
	return this->lastUpdateTime;
}

AU_UAV_ROS::waypoint AU_UAV_ROS::PlaneObject::getDestination(void) const {
	return this->destination;
}

/* Find distance between this plane and another plane, returns in meters */
double AU_UAV_ROS::PlaneObject::findDistance(const AU_UAV_ROS::PlaneObject& plane) const {
	return this->findDistance(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find distance between this plane and another pair of coordinates, 
returns value in meters */
double AU_UAV_ROS::PlaneObject::findDistance(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;

	return sqrt(pow(xdiff, 2) + pow(ydiff, 2));
}

/* Find Cartesian angle between this plane and another plane, using this plane
as the origin */
double AU_UAV_ROS::PlaneObject::findAngle(const AU_UAV_ROS::PlaneObject& plane) const {
	return this->findAngle(plane.currentLoc.latitude, plane.currentLoc.longitude);
}

/* Find Cartesian angle between this plane and another plane's latitude/longitude 
using this plane as the origin */
double AU_UAV_ROS::PlaneObject::findAngle(double lat2, double lon2) const {
	double xdiff = (lon2 - this->currentLoc.longitude)*DELTA_LON_TO_METERS;
	double ydiff = (lat2 - this->currentLoc.latitude)*DELTA_LAT_TO_METERS;
	
	return atan2(ydiff, xdiff);
}


AU_UAV_ROS::PlaneObject& AU_UAV_ROS::PlaneObject::operator=(const AU_UAV_ROS::PlaneObject& plane) {

	this->id = plane.id;
	this->currentLoc.altitude = plane.currentLoc.altitude;
	this->currentLoc.latitude = plane.currentLoc.latitude;
	this->currentLoc.longitude = plane.currentLoc.longitude;

	this->previousLoc.altitude = plane.previousLoc.altitude;
	this->previousLoc.latitude = plane.previousLoc.latitude;
	this->previousLoc.longitude = plane.previousLoc.longitude;

	this->destination.latitude = plane.destination.latitude;
	this->destination.longitude = plane.destination.longitude;
	this->destination.altitude = plane.destination.latitude;

	this->targetBearing = plane.targetBearing;
	this->currentBearing = plane.currentBearing;

	this->speed = plane.speed;
	this->lastUpdateTime = plane.lastUpdateTime;
	this->collisionRadius = plane.collisionRadius;

	return *this;
}
