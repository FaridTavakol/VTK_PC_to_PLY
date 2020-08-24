//============================================================================
// Name        : ProstateKinematics.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the custom forward and inverse kinematics
//				 code for the robot resides
//============================================================================

#ifndef PROSTATEKINEMATICS_HPP_
#define PROSTATEKINEMATICS_HPP_

#include <math.h>
#include "..\..\Kinematics\Kinematics.hpp"
#include "Eigen\Dense"

struct Prostate_FK_outputs {
	Eigen::Matrix4d BaseToTreatment;
	double xNeedleTip;
	double yNeedleTip;
	double zNeedleTip;
};

struct Prostate_IK_outputs {
	double xFrontSlider1;
	double xFrontSlider2;
	double xRearSlider1;
	double xRearSlider2;
	double zInsertion;
	double zRotation;
};

class ProstateKinematics : public Kinematics {


public:
	ProstateKinematics();

	// Inputs to the Forward and Inverse  Kinematics are given in units, not ticks
	Prostate_FK_outputs ForwardKinematics(double xFrontSlider1, double xFrontSlider2, double xRearSlider1, double xRearSlider2, double zInsertion); //in units
	Prostate_IK_outputs InverseKinematics(double xNeedleDesired, double yNeedleDesired, double zNeedleDesired); //in units

	//Robot Specific Parameters
	double _lengthTrapSideLink;
	double _widthTrapTop;
	double _heightLowerTrapOffset;
	double _heightUpperTrapOffset;
	double _lengthNeedleTipOffset;
	double _distanceBetweenTraps;

	//**Values that update with motion**//
	//Trapazoid points of rotation
	double _xFrontPointOfRotation;
	double _yFrontPointOfRotation;
	double _zFrontPointOfRotation;

	double _xRearPointOfRotation;
	double _yRearPointOfRotation;
	double _zRearPointOfRotation;

	//Angulation Variables
	double _alpha;
	double _beta; //updated continuously
	double _C; //FOR NOW.. probably changes with angulation; where C is the distance between the point of rotation and center of the front trapezoid stage in the Z-direction,
	double _h; //FOR NOW.. probably changes wiht angulation; h is the distance between the needle’s direction and	the center of the front trapezoid stage in the vertical direction

};

#endif /* PROSTATEKINEMATICS_HPP_ */
