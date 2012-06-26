/*
Implementation of vmath.h.  For information on how to use these functions, visit vmath.h.  Comments in this file
are related to implementation, not usage.
*/

#include <math.h>
#include "sim/vmath.h"
#include "sim/standardFuncs.h" /* for PI */

/* Constructor - specify magnitude and direction of the force vector */
sim::mathVector::mathVector(double m, double d) :
	magnitude(m), direction(d) {}

/* Copy constructor */
sim::mathVector::mathVector(const sim::mathVector& mV) {
	this->direction = mV.direction;
	this->magnitude = mV.magnitude;
}

/* Accessor methods: Allow the client to access the magnitude and direction of the math vector */
double sim::mathVector::getDirection(void) const {
	return direction;
}

double sim::mathVector::getMagnitude(void) const {
	return magnitude;
}

/* Modifier methods: Allow the client to modify the magnitude and direction of the math vector */
void sim::mathVector::setDirection(double d) {
	this->direction = d;
}

void sim::mathVector::setMagnitude(double m) {
	this->magnitude = m;
}

/*
Vector-based addition, subtract, and multiplication operator overloading.
*/

/* Dot produt. */ 
double sim::mathVector::dotProduct(const sim::mathVector& mV){
	/* Ax = mag * cos(degree), Ay = mag * sin(degree) */
	double Ax = this->magnitude * cos(this->direction * PI / 180.0);
	double Ay = this->magnitude * sin(this->direction * PI / 180.0);

	/* Bx = mag * cos(degree), By = mag * sin(degree) */
	double Bx = mV.magnitude * cos(mV.direction * PI / 180.0);
	double By = mV.magnitude * sin(mV.direction * PI / 180.0);

	return Ax*Bx+Ay*By;

}

const sim::mathVector sim::mathVector::operator+(const sim::mathVector& mV) const {
	mathVector newVector(*this); /* Copy vector */
	newVector += mV; /* Use += to assign */
	return newVector;
}

sim::mathVector& sim::mathVector::operator+=(const sim::mathVector& mV) {
	double Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0;
	double Rx = 0.0, Ry = 0.0, R = 0.0, Theta = 0.0;

	/* Ax = mag * cos(degree), Ay = mag * sin(degree) */
	Ax = this->magnitude * cos(this->direction * PI / 180.0);
	Ay = this->magnitude * sin(this->direction * PI / 180.0);

	/* Bx = mag * cos(degree), By = mag * sin(degree) */
	Bx = mV.magnitude * cos(mV.direction * PI / 180.0);
	By = mV.magnitude * sin(mV.direction * PI / 180.0);

	/* Perform addition */
	Rx = Ax + Bx;
	Ry = Ay + By;

	/* R = (Rx^2 + Ry^2)^(1/2) */
	R = sqrt(pow(Rx, 2) + pow(Ry, 2));

	/* Theta = arctan2(Ry, Rx) */
	Theta = atan2(Ry , Rx) * 180.0 / PI;

	/* Assign new values */
	this->magnitude = R;
	this->direction = Theta;

	/* Return resulting vector */
	return *this;
}

const sim::mathVector sim::mathVector::operator-(const sim::mathVector& mV) const {
	mathVector newVector(*this);
	newVector -= mV;
	return newVector;
}

sim::mathVector& sim::mathVector::operator-=(const sim::mathVector& mV) {
	double Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0;
	double Rx = 0.0, Ry = 0.0, R = 0.0, Theta = 0.0;

	/* Ax = mag * cos(degree), Ay = mag * sin(degree) */
	Ax = this->magnitude * cos(this->direction * PI / 180.0);
	Ay = this->magnitude * sin(this->direction * PI / 180.0);

	/* -Bx = mag * cos(-degree), -By = mag * sin(-degree) */
	Bx = mV.magnitude * cos((mV.direction + 180.0) * PI / 180.0);
	By = mV.magnitude * sin((mV.direction + 180.0) * PI / 180.0);

	/* Perform addition with negative vector */
	Rx = Ax + Bx;
	Ry = Ay + By;

	/* R = (Rx^2 + Ry^2)^(1/2) */
	R = sqrt(pow(Rx, 2) + pow(Ry, 2));

	/* Theta = arctan2(Ry, Rx) */
	Theta = atan2(Ry , Rx) * 180.0 / PI;

	/* Assign new values */
	this->magnitude = R;
	this->direction = Theta;

	/* Return resulting vector */
	return *this;
}

/*
Double-based operator overloading for multiplication and division.
*/

const sim::mathVector sim::mathVector::operator*(double val) const {
	mathVector newVector(*this);
	newVector *= val;
	return newVector;
}

sim::mathVector& sim::mathVector::operator*=(double val) {
	/* Scalar multiplication involves multiplying the magnitude by the scale */
	this->magnitude *= val;

	/* Return resulting vector */
	return *this;
}

const sim::mathVector sim::mathVector::operator/(double val) const {
	mathVector newVector(*this);
	newVector /= val;
	return newVector;
}

sim::mathVector& sim::mathVector::operator/=(double val) {
	/* Scalar division involves dividing the magnitude by the scale */
	this->magnitude /= val;

	/* Return resulting vector */
	return *this;
}

/* Overloaded multiplication operator.  Multiplies the magnitude of the vector by a constant */
const sim::mathVector operator*(double val, const sim::mathVector& mV){
	sim::mathVector newVector(mV);
  	newVector *= val;
  	return newVector;
}

/* Overloaded division operator.  Divides the magnitude of the vector by a constant. */
const sim::mathVector operator/(double val, const sim::mathVector& mV){
  	sim::mathVector newVector(mV);
  	newVector /= val;
  	return newVector;
}


/* Overloaded equality operator */
sim::mathVector& sim::mathVector::operator=(const sim::mathVector& mV){
	this->magnitude = mV.magnitude;
	this->direction = mV.direction;
	return *this;
}


