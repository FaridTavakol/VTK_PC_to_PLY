//============================================================================.
// Name        : NeuroRobot.hpp.
// Author      : Produced in the WPI AIM Lab.
// Description : This file defines the methods and parameters of the class Neurosurgery Robot.
//============================================================================

#ifndef NEUROROBOT_HPP_
#define NEUROROBOT_HPP_

using namespace std;
// #include "..\..\Robots\Robot.hpp"
// #include "..\..\Utilities\FPGA\FPGA_Utilities.hpp"
// #include "..\..\Utilities\Packets\Packets.hpp"
#include "NeuroKinematics/NeuroKinematics.hpp"
#include <string>

class NeuroRobot : public Robot
{
public:
    //================ Constructor ================
    NeuroRobot(Packets *packets, FPGA_Utilities *fpga_util, int loopRate);
    virtual ~NeuroRobot(){};

    //================ Parameters =================
    // Parameters are initialized in constructor
    // See the Robot.hpp file for declaration of robot specific variables such as : _name, _registration, _probe, etc
    // See the NeuroRobot.cpp file for implementation of these robot specific variables for the NeuroRobot

    // Packet Information for SPI Communications
    Packets *_packets;

    // The FPGA Utility object was included to allow for control over LEDs
    FPGA_Utilities *_fpga_util;

    // Timing related variables
    Timer _timer;
    int _loopRate;

    // Frequency Sweep specific variable
    int _frequency;

    //=================== Neuro Robot Kinematics ==================
    // This class contains the forward and inverse kinematics for this robot
    NeuroKinematics _neuroKinematics;

    //=================== NeuroRobot Motors =======================
    // NeuroRobot Motors are defined below:
    // -- Insertion Motors
    Motor_Config ProbeInsertionConfig;
    Motor _probeInsertion;

    // -- Orientation Motors
    Motor_Config YawRotationConfig, PitchRotationConfig, ProbeRotationConfig;
    Motor _yawRotation, _pitchRotation, _probeRotation;

    // -- Axial Motors
    Motor_Config AxialHeadTranslationConfig, AxialFeetTranslationConfig, LateralTranslationConfig, PlasticMotorConfig;
    Motor _axialHeadTranslation, _axialFeetTranslation, _lateralTranslation;

    //================ Public Methods ==============
    // Methods that must be implemented from the abstract Robot Class
    void Update(); // This is the central robot functionality method, and this is called from our main loop in SurgicalRobot.cpp
    void HomeRobot();
    void StopRobot();
    void ZeroRobot();
    void UpdateRobot(int usPeriod);
    void RunInverseKinematics(int options);
    void Axis_Setpoint_Validator();
    int IsFootPedalPressed();
    Motor *GetMotor(int cardID);
    vector<Motor *> ListMotors();
    bool CheckForStalls();

    // Other Methods specific only to the NeuroRobot
    void FollowTrajectory();
    void FrequencySweep(int cardID);
};

#endif /* NEUROROBOT_HPP_ */
