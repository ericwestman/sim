//////////////////////////////////////////////////////////////////////////////////////
////////////////////SHOULD TURN RIGHT//////////////////////////////////////////////	

	/* For checking whether the plane should turn right or left */
	double theta, theta_prev, theta_prime, delta_theta, theta1, theta2;
	bool turnRight, plane2OnRight, plane1OnRight;
	double cartBearing1 = toCartesian(plane1.getCurrentBearing());
	double cartBearing2 = toCartesian(plane2.getCurrentBearing());

	/* Calculate theta, theta1, and theta2. Theta is the cartesian angle
	from 0 degrees (due East) to plane2 (using plane1 as the origin). This 
	may be referred to as the LOS angle. */
	theta = findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, 
		plane2.getCurrentLoc().latitude, plane2.getCurrentLoc().longitude);
	theta_prev = findAngle(plane1.getPreviousLoc().latitude, plane1.getPreviousLoc().longitude, 
		plane2.getPreviousLoc().latitude, plane2.getPreviousLoc().longitude);
	delta_theta = theta - theta_prev;
	theta_prime = manipulateAngle(theta+180.0);
	theta1 = manipulateAngle(cartBearing1 - theta);
	theta2 = manipulateAngle(cartBearing2 - theta_prime);

	/* Calculate which side of plane1 that plane2 is on, and also which side of plane2 plane 1 is on.*/
	plane2OnRight = theta1 >= 0; plane1OnRight = theta2 >= 0;

	/* Set theta1 to be on positive*/
	theta1 = fabs(theta1);
	theta2 = fabs(theta2);

	//ROS_WARN("Plane %d: Bearing: %f plane2OnRight: %d", plane1.getID(), plane1.getCurrentBearing(), plane2OnRight);

	/* Calculate which direction to turn*/
	if (plane2OnRight && plane1OnRight) turnRight = false;
	else if (!plane2OnRight && !plane1OnRight) turnRight = true;
	else if (plane2OnRight && !plane1OnRight) turnRight = (theta1 <= theta2) ? true : false;
	else if (!plane2OnRight && plane1OnRight) turnRight = (theta1 >= theta2) ? true : false;

	if (plane1.getID() == 2 || plane2.getID() == 2)
		//ROS_WARN("Plane %d: %f | Plane %d: %f TurnRight: %d", plane1.getID(), theta1, plane2.getID(), theta2, turnRight);
	return turnRight;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Calculate Waypoint///////////////////////////////////////////////////////



	AU_UAV_ROS::coordinate turningCircleCenter = calculateCircleCenter(plane1, turningRadius, turnRight);
	AU_UAV_ROS::waypoint wp;	
	double maxTurnAngle = PLANE_MAX_TURN_ANGLE;
	double r0 = turningRadius; //radius of turning circle
	double r1 = MPS_SPEED*TIME_STEP; //distance travelled in one time step
	double d = turningRadius; //distance between circle centers
	double a = (pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2 * d);
	double h = sqrt(pow(r0, 2) - pow(a, 2));

	// Using turningCircleCenter as origin
	double x0 = 0;
	double y0 = 0;

	double x1 = (plane1.getCurrentLoc().longitude - turningCircleCenter.longitude) * DELTA_LON_TO_METERS;
	double y1 = (plane1.getCurrentLoc().latitude - turningCircleCenter.latitude) * DELTA_LAT_TO_METERS;

	double x2 = x0 + a * (x1 - x0) / d;
	double y2 = y0 + a * (y1 - y0) / d;

	double x3 = x2 + h * (y1 - y0) / d;
	double y3 = y2 - h * (x1 - x0) / d;

	if (fabs(manipulateAngle(toCartesian(plane1.getCurrentBearing()) - atan2((y3-y1),(x3-x1))*180/PI)) > maxTurnAngle) {
		x3 = x2 - h * (y1 - y0) / d;
		y3 = y2 + h * (x1 - x0) / d;
	}

	wp.latitude = turningCircleCenter.latitude + y3/DELTA_LAT_TO_METERS;
	wp.longitude = turningCircleCenter.longitude + x3/DELTA_LON_TO_METERS;
	wp.altitude = plane1.getCurrentLoc().altitude;	


	ROS_WARN("Plane%d new cbearing: %f", plane1.getID(), toCardinal( findAngle(plane1.getCurrentLoc().latitude, plane1.getCurrentLoc().longitude, wp.latitude, wp.longitude) ) ); 

	
	return wp;




