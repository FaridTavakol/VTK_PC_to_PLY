#pragma once
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include "../../NeuroKinematics/NeuroKinematics.hpp"

class ForwardKinematics
{

public:
    ForwardKinematics(NeuroKinematics &NeuroKinematics);

    //members
    double i, j, k, l; //counter initialization
                       // Min allowed seperation 75mm
    // Max allowed seperation  146mm
    const double Diff; // Is the max allowed movement while one block is stationary 146-75 = 71 mm
    const double pi;
    double Ry;      // Initializing the PitchRotation counter
    double RyF_max; // in paper is 37.2
    double RyB_max; // in paper is  30.6
    double Rx;      // Initializing the YawRotation counter
    double Rx_max;  // Max YawRotation
    // Robot axis
    double AxialHeadTranslation;
    double AxialFeetTranslation;
    double LateralTranslation;
    double PitchRotation;
    double YawRotation;
    double ProbeInsertion;
    double ProbeRotation;
    NeuroKinematics NeuroKinematics_;

    //methods
    vtkSmartPointer<vtkPoints> get_General_Workspace(Eigen::Matrix4d registration);
    vtkSmartPointer<vtkPoints> get_Sub_Workspace(Eigen::Matrix4d registration, Eigen::Vector4d entryPointScanner);
    void nan_ckecker(Neuro_FK_outputs FK);
};