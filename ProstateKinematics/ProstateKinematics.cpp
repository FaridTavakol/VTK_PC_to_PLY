//============================================================================
// Name        : ProstateKinematics.cpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the custom forward and inverse kinematics
//				 code for the robot resides
//============================================================================

#include "ProstateKinematics.hpp"

ProstateKinematics::ProstateKinematics() {

	//Robot Specific Parameters
	_lengthTrapSideLink = 124;
	_widthTrapTop = 84;
	_heightLowerTrapOffset = 12;
	_heightUpperTrapOffset = 67.5;
	//_lengthNeedleTipOffset = 259.075;
	_lengthNeedleTipOffset = 259.075;
	_distanceBetweenTraps = 181.5;

	//**Values that update with motion**//
	//Trapazoid points of rotation
	_xFrontPointOfRotation = 0;
	_yFrontPointOfRotation = 0;
	_zFrontPointOfRotation = 0;

	_xRearPointOfRotation = 0;
	_yRearPointOfRotation = 0;
	_zRearPointOfRotation = 0;

	//Angulation Variable
	_alpha = 0;
	_beta = 0; //updated continuously
	_C = 0; //FOR NOW.. probably changes with angulation; where C is the distance between the point of rotation and center of the front trapezoid stage in the Z-direction,
	_h = 0; //FOR NOW.. probably changes wiht angulation; h is the distance between the needle’s direction and	the center of the front trapezoid stage in the vertical direction
}

Prostate_FK_outputs ProstateKinematics::ForwardKinematics(double xFrontSlider1, double xFrontSlider2, double xRearSlider1, double xRearSlider2, double zInsertion) {

	struct Prostate_FK_outputs FK;

	//*** BASE FORWARD KINEMATICS ***//
	_xFrontPointOfRotation = (xFrontSlider1 + xFrontSlider2)/2;

	double yF_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yF_2 = pow(_lengthTrapSideLink,2);
	double yF_3 = pow((xFrontSlider1-xFrontSlider2-_widthTrapTop)/2,2);
	_yFrontPointOfRotation = yF_1 + sqrt(yF_2 - yF_3);

	_zFrontPointOfRotation = -_C;

	_xRearPointOfRotation = (xRearSlider1 + xRearSlider2)/2;
	double yR_1 = _heightLowerTrapOffset + _heightUpperTrapOffset;
	double yR_2 = pow(_lengthTrapSideLink,2);
	double yR_3 = pow((xRearSlider1-xRearSlider2-_widthTrapTop)/2,2);
	_yRearPointOfRotation = yR_1 + sqrt(yR_2-yR_3);

	_zRearPointOfRotation = 0;

	_alpha = 0;//atan2(_xFrontPointOfRotation-_xRearPointOfRotation, _distanceBetweenTraps);
	_beta = 0;//atan2(_yFrontPointOfRotation - _yRearPointOfRotation, _distanceBetweenTraps);

	FK.xNeedleTip = (_lengthTrapSideLink + zInsertion)*cos(_beta)*sin(_alpha) + _h*sin(_beta)*sin(_alpha) + _xFrontPointOfRotation;

	FK.yNeedleTip = _h*cos(_beta) - (_lengthTrapSideLink + zInsertion)*sin(_beta) + _yFrontPointOfRotation;

	//*** NEEDLE DRIVER FORWARD KINEMATICS ***//
	FK.zNeedleTip = (_lengthNeedleTipOffset + zInsertion)*cos(_beta)*cos(_alpha) + _h*sin(_beta)*cos(_alpha) + _zFrontPointOfRotation;

	FK.BaseToTreatment <<   1,  0,  0, FK.xNeedleTip,
						    0,  1,  0, FK.yNeedleTip,
						    0,  0,  1, FK.zNeedleTip,
						    0,  0,  0,  1;

	return FK;

}

Prostate_IK_outputs ProstateKinematics::InverseKinematics(double xNeedleDesired, double yNeedleDesired, double zNeedleDesired) {

	struct Prostate_IK_outputs IK;

	//*** BASE INVERSE KINEMATICS **//
	double xFrontPointOfRotationDesired = xNeedleDesired;
	double yFrontPointOfRotationDesired = yNeedleDesired;

	double xRearPointOfRotationDesired = xNeedleDesired;
	double yRearPointOfRotationDesired = yNeedleDesired;

	//Calculation for Front Slider 1 and 2
	double xF1_1 = 2*xFrontPointOfRotationDesired + _widthTrapTop;
	double xF1_2 = pow(_lengthTrapSideLink,2);
	double xF1_3 = pow(yFrontPointOfRotationDesired - _heightLowerTrapOffset - _heightUpperTrapOffset,2);

	IK.xFrontSlider1 = 0.5*(xF1_1 + 2*sqrt(xF1_2 - xF1_3));
	IK.xFrontSlider2 = 2*xFrontPointOfRotationDesired - IK.xFrontSlider1;

	//Calculation for Rear Slider 1 and 2
	double xR1_1 = 2*xRearPointOfRotationDesired + _widthTrapTop;
	double xR1_2 = pow(_lengthTrapSideLink,2);
	double xR1_3 = pow(yRearPointOfRotationDesired - _heightLowerTrapOffset - _heightUpperTrapOffset,2);

	IK.xRearSlider1 = 0.5*(xR1_1 + 2*sqrt(xR1_2 - xR1_3));
	IK.xRearSlider2 = 2*xRearPointOfRotationDesired - IK.xRearSlider1;

	//*** NEEDLE DRIVER INVERSE KINEMATICS ***//
	IK.zInsertion =  zNeedleDesired -_lengthNeedleTipOffset;
	IK.zRotation = 0; //FROM OPENIGTLINKTRACKING

	return IK;
}

