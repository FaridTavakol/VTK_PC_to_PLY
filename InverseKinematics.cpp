#include "NeuroKinematics/NeuroKinematics.hpp"
#include <iostream>
#include <string>
#include <math.h> /* isnan, sqrt */
#include <cstdlib>
#include <eigen3/Eigen/Dense>
using namespace std;

//=================== Neuro Robot Poses ======================
// Transformation from scanner to robot zFrame
//_registration = Eigen::Matrix4d::Identity();
double _cannulaToTreatment{5.0};
double _treatmentToTip{10.0};
double _robotToEntry{5.0};
double _robotToTreatmentAtHome{41.0};
double AxialHeadTranslation{0.0};
double AxialFeetTranslation{0.0};
double LateralTranslation{0.0};
double PitchRotation{0.0};
double YawRotation{0.0};
double ProbeInsertion{0.0};
double ProbeRotation{0.0};

int main()
{
    Probe probe1 = {_cannulaToTreatment, _treatmentToTip, _robotToEntry, _robotToTreatmentAtHome};
    Probe *_probe{&probe1}; // Creating a pointer Probe1 of type Probe that points to the address of probe_init
    NeuroKinematics Inverse(_probe);
    // Current and Target pose are defaulted to identity
    // These transformations are with respect to the Imager Coordinate Frame
    Eigen::Matrix4d _currentPose;
    Eigen::Matrix4d _targetPose;
    _currentPose = Eigen::Matrix4d::Identity();
    _targetPose = Eigen::Matrix4d::Identity();
    // An arbitrary Registration matrix is selected. This matrix is dependant of the Imager
    Eigen::Matrix4d _registration;
    _registration = (Eigen::Matrix4d() << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1)
                        .finished();

    // Desired values for the entry and target points ( This is hardcoded now, in future it should be a cin or recieved from the Slicer).
    Eigen::Vector3d e1(18.32, 245, 0);
    Eigen::Vector3d t1(18.32, 244, 0);
    // Eigen::Vector3d e1(-176, 260.8, -2);
    // Eigen::Vector3d t1(-176, 250.8, -2);
    Eigen::Vector3d _entryPoint;  // Entry point for the desired pass-through location of the treatment zone
    Eigen::Vector3d _targetPoint; // Target point for the final location of the treatment zone
    _entryPoint = e1;
    _targetPoint = t1;

    // Entry and Target Points are received in scanner coordinates
    // Multiply them by the registration matrix to obtain them in Robot's zFrame coordinates
    Eigen::Vector4d entryPointScannerCoordinates(_entryPoint(0), _entryPoint(1), _entryPoint(2), 1);
    Eigen::Vector4d targetPointScannerCoordinates(_targetPoint(0), _targetPoint(1), _targetPoint(2), 1);

    // Calculate zFrameToEntry
    Eigen::Vector4d zFrameToEntry = _registration.inverse() * entryPointScannerCoordinates;
    // Calculate zFrameToTarget
    Eigen::Vector4d zFrameToTarget = _registration.inverse() * targetPointScannerCoordinates;

    // Perform Inverse Kinematics
    // Input results into the InverseKinematics which expects entry point and target point to be with respect to the zFrame
    // This is divided into two Steps:
    // First step: 1) the first step asks for the entry point and returns the FeetAxial HeadAxial, and Lateral translation values.
    // 2) the values are sent to the FK and the subset of the workspace is created
    // Second step: 1) The target point is recieved. The Full IK is derived and FK values are generated and sent to the robot.
    //Firts step:

    // Neuro_IK_outputs IK{}; // Object IK for the first step is created.
    // // XYZ components of the desired Entry point in the Robot's zframe
    // double XEntry = zFrameToEntry(0);
    // double YEntry = zFrameToEntry(1);
    // double ZEntry = zFrameToEntry(2);
    // //------------------------------
    // // Get the entry point with respect to the orientation of the zFrame
    // Eigen::Vector4d rcmToEntry = Inverse._zFrameToRCM.inverse() * zFrameToEntry;

    // // Get the target point with respect to the orientation of the zFrame
    // Eigen::Vector4d rcmToTarget = Inverse._zFrameToRCM.inverse() * zFrameToTarget;

    // // The yaw and pitch components of the robot rely solely on the entry point's location with respect to the target point
    // // This calculation is done with respect to the RCM Orientation
    // IK.YawRotation = (3.1415 / 2) + atan2(rcmToEntry(2) - rcmToTarget(2), rcmToEntry(1) - rcmToTarget(1));
    // IK.PitchRotation = atan((rcmToEntry(0) - rcmToTarget(0)) / (rcmToEntry(2) - rcmToTarget(2)));
    // //------------------------------

    // // The Lateral Translation is given by the desired distance in x
    // IK.LateralTranslation = XEntry - Inverse._xInitialRCM - Inverse._robotToRCMOffset * sin(IK.PitchRotation) + _probe->_robotToEntry * sin(IK.PitchRotation);
    // // Equations calculated through the symbolic equations for the Forward Kinematics
    // // Substituting known values in the FK equations yields the value for Axial Head and Feet
    // IK.AxialHeadTranslation = ZEntry - Inverse._initialAxialSeperation / 2 + Inverse._widthTrapezoidTop / 2 - Inverse._zInitialRCM + sqrt(8 * YEntry * Inverse._yInitialRCM - 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop - 4 * YEntry * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) + 4 * Inverse._yInitialRCM * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(Inverse._initialAxialSeperation, 2) + pow(Inverse._widthTrapezoidTop, 2) - 4 * pow(Inverse._yInitialRCM, 2) - 4 * pow(Inverse._robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * Inverse._robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * Inverse._robotToRCMOffset * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2))) / 2 + Inverse._robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);
    // IK.AxialFeetTranslation = ZEntry + Inverse._initialAxialSeperation / 2 - Inverse._widthTrapezoidTop / 2 - Inverse._zInitialRCM - sqrt(8 * YEntry * Inverse._yInitialRCM - 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop - 4 * YEntry * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) + 4 * Inverse._yInitialRCM * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(Inverse._initialAxialSeperation, 2) + pow(Inverse._widthTrapezoidTop, 2) - 4 * pow(Inverse._yInitialRCM, 2) - 4 * pow(Inverse._robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * Inverse._robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * Inverse._robotToRCMOffset * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * Inverse._yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * Inverse._robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(Inverse._initialAxialSeperation, 2) + 2 * Inverse._initialAxialSeperation * Inverse._widthTrapezoidTop + 4 * pow(Inverse._lengthOfAxialTrapezoidSideLink, 2) - pow(Inverse._widthTrapezoidTop, 2))) / 2 + Inverse._robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);

    // // Output is displayed based on the IK calculations
    // std::cout << "======================IK calculations in 1'st step=========================" << endl;
    // std::cout << "Desired Lateral translation : " << IK.LateralTranslation << endl;
    // std::cout << "Desired AxialHead translation :  " << IK.AxialHeadTranslation << endl;
    // std::cout << "Desired AxialFeet translation :  " << IK.AxialFeetTranslation << endl;

    // std::cout << "======================FK Output Based on the IK from the first step=========================" << endl;
    // AxialFeetTranslation = IK.AxialFeetTranslation;
    // AxialHeadTranslation = IK.AxialHeadTranslation;
    // LateralTranslation = IK.LateralTranslation;

    // PitchRotation = IK.PitchRotation;
    // YawRotation = IK.YawRotation;
    // ProbeInsertion = 0.0;
    // ProbeRotation = 0.0;
    // Neuro_FK_outputs FK = Inverse.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
    //                                                 LateralTranslation, ProbeInsertion,
    //                                                 ProbeRotation, PitchRotation, YawRotation);

    // std::cout << "X pos of the end-effector :" << FK.zFrameToTreatment(0, 3) << endl;
    // std::cout << "Y pos of the end-effector :" << FK.zFrameToTreatment(1, 3) << endl;
    // std::cout << "Z pos of the end-effector :" << FK.zFrameToTreatment(2, 3) << endl;
    // std::cout << "desired entry point: \n"
    //           << zFrameToEntry << endl;

    // std::cout << "======================End of FK Output=========================" << endl;

    // ==================================================
    Neuro_IK_outputs IK_output = Inverse.InverseKinematics(zFrameToEntry, zFrameToTarget);
    _targetPose = _registration * (Eigen::Matrix4d() << IK_output.targetPose).finished();

    // Printing stuff
    std::cout << "Target Pose is : \n"
              << _targetPose << endl;
    std::cout << "Desired Lateral translation : " << IK_output.LateralTranslation << endl;
    std::cout << "Desired AxialHead translation :  " << IK_output.AxialHeadTranslation << endl;
    std::cout << "Desired AxialFeet translation :  " << IK_output.AxialFeetTranslation << endl;
    std::cout << "Desired ProbeInsertion is : " << IK_output.ProbeInsertion << endl;
    std::cout << "Desired Probe Rotation is : " << IK_output.ProbeRotation << endl;
    std::cout << "Desired Yaw Rotation is : " << IK_output.YawRotation << endl;
    std::cout << "Desired Pitch Rotation is : " << IK_output.PitchRotation << endl;
    std::cout << "Location of the selected Entry point in the Robot's Cartesian Space : \n"
              << zFrameToEntry << endl;
    std::cout << "======================FK Output=========================" << endl;

    AxialFeetTranslation = IK_output.AxialFeetTranslation;
    AxialHeadTranslation = IK_output.AxialHeadTranslation;
    LateralTranslation = IK_output.LateralTranslation;
    ProbeInsertion = IK_output.ProbeInsertion;
    ProbeRotation = IK_output.ProbeRotation;
    PitchRotation = IK_output.PitchRotation;
    YawRotation = IK_output.YawRotation;
    Neuro_FK_outputs FK3 = Inverse.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                     LateralTranslation, ProbeInsertion,
                                                     ProbeRotation, PitchRotation, YawRotation);

    std::cout << "Probe Insertion Value :" << IK_output.ProbeInsertion << endl;

    std::cout << "X Position :" << FK3.zFrameToTreatment(0, 3) << endl;
    std::cout << "Y Position :" << FK3.zFrameToTreatment(1, 3) << endl;
    std::cout << "Z Position :" << FK3.zFrameToTreatment(2, 3) << endl;
    std::cout << "zFrameToTreatment :\n"
              << FK3.zFrameToTreatment << endl;

    ProbeInsertion = 0;
    std::cout << "Probe Insertion Value is now set to:" << ProbeInsertion << endl;
    std::cout << "======================Updated FK Output=========================" << endl;
    Neuro_FK_outputs FK1 = Inverse.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                     LateralTranslation, ProbeInsertion,
                                                     ProbeRotation, PitchRotation, YawRotation);

    std::cout << "X entry Position :" << FK1.zFrameToTreatment(0, 3) << endl;
    std::cout << "Y entry Position :" << FK1.zFrameToTreatment(1, 3) << endl;
    std::cout << "Z entry Position :" << FK1.zFrameToTreatment(2, 3) << endl;
    std::cout << "zFrameToTreatment :\n"
              << FK1.zFrameToTreatment << endl;
    // Axis_Setpoint_Validator();
    return 0;
}