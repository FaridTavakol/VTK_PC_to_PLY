#include "ForwardKinematics.h"
using std::endl;
using std::ofstream;

// A is treatment to tip, where treatment is the piezoelectric element,// A = 10mm
// B is robot to entry, this allows us to specify how close to the patient the physical robot can be,// B = 5mm
// C is cannula to treatment, we define this so the robot can compute the cannula length,// C = 5mm
// D is the robot to treatment distance,// D = 41mm
// Creating an object called Forward for FK
// In the neuroRobot.cpp the specs for the  probe are: 0,0,5,41

ForwardKinematics::ForwardKinematics(NeuroKinematics &NeuroKinematics) : Diff(68), pi(3.141)
{
  //counters
  i = 0.0;
  j = 0.0;
  k = 0.0;
  l = 0.0;
  ii = 0.0;
  // Min allowed seperation 75mm
  // Max allowed seperatio1f46mm
  Ry = 0.0;                   // Initializing the PitchRotation counter
  RyF_max = -37.0 * pi / 180; // in paper is 37.2
  RyF_max_degree = -37.0;
  RyB_max = +26.0 * pi / 180; // in paper is  30.6
  RyB_max_degree = 26.0;
  Rx = 0.0;                  // Initializing the YawRotation counter
  Rx_max = -88.0 * pi / 180; // Max YawRotation
  Rx_max_degree = -88.0;
  // Robot axis
  AxialHeadTranslation = 0.0;
  AxialFeetTranslation = 0.0;
  LateralTranslation = 0.0;
  PitchRotation = 0.0;
  YawRotation = 0.0;
  ProbeInsertion = 0.0;
  ProbeRotation = 0.0;

  NeuroKinematics_ = NeuroKinematics;
}

vtkSmartPointer<vtkPoints> ForwardKinematics::get_General_Workspace(Eigen::Matrix4d registration)
{
  // To visualize the transferred points in the slicer without using the Transform Module
  Eigen::Matrix4d registration_inv = registration.inverse();
  std::cerr << "Inverse of the registration matrix is: " << registration_inv << std::endl;

  // Vector to store points before transformation
  Eigen::Vector4d point(0.0, 0.0, 0.0, 0.0);

  // Object containing the 4x4 transformation matrix
  Neuro_FK_outputs FK{};

  // Create points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  /*============================================================================================================
     =============================================FK computation============================================
      ==================================================================================================*/
  // Loop for visualizing the top
  for (i = 0.0, j = 68.0; i > -146.0; i -= 14.5, j -= 14.5) // initial separation 143, min separation 75 => 143-75 = 68 mm
  {
    AxialHeadTranslation = i;
    AxialFeetTranslation = j;
    for (k = 0.0; k <= -49.0; k -= 7.0) // max lateral movement 0.0 ~ -49.47 (appx = -49)
    {
      LateralTranslation = k;
      FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                              LateralTranslation, ProbeInsertion,
                                              ProbeRotation, PitchRotation, YawRotation);
      nan_checker(FK);
      Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
      transferred_point = get_Transform(registration_inv, FK);
      points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
    }
  }

  // loop for visualizing the bottom
  for (i = 0, j = -3; i > -87; i -= 8.7, j -= 8.7) //75
  {
    AxialHeadTranslation = i;
    AxialFeetTranslation = j;
    for (k = 0.0; k <= -49; k -= 7.0)
    {
      LateralTranslation = k;
      if (k == -49.0)
      {

        YawRotation = Rx_max;
        PitchRotation = RyF_max;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        nan_checker(FK);
        Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
        transferred_point = get_Transform(registration_inv, FK);
        points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
      }
      else
      {
        YawRotation = Rx_max;
        PitchRotation = RyB_max;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        nan_checker(FK);
        Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
        transferred_point = get_Transform(registration_inv, FK);
        points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
      }
    }
  }

  YawRotation = 0;
  PitchRotation = 0;
  AxialFeetTranslation = -3;
  AxialHeadTranslation = 0;

  // Loop for creating the head face
  for (j = 7.1; j <= 71; j += 7.1)
  {
    AxialFeetTranslation += j;

    for (k = 0.0; k <= -49.0; k -= 7.0)
    {
      LateralTranslation = k;

      if (j == 71) // Top level
      {
        if (k == 0)
        {
          for (i = 0; i <= RyB_max_degree; i += 5.2)
          {
            PitchRotation = i * pi / 180;
            for (ii = 0; ii >= Rx_max_degree; ii -= 8.8)
            {
              YawRotation = ii * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_checker(FK);
              Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
              transferred_point = get_Transform(registration_inv, FK);
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
        }
        else if (k == -49.0)
        {
          for (i = 0; i >= RyF_max_degree; i -= 7.4)
          {
            PitchRotation = i * pi / 180;
            for (ii = 0; ii >= Rx_max_degree; ii -= 8.8)
            {
              YawRotation = ii * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_checker(FK);
              Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
              transferred_point = get_Transform(registration_inv, FK);
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
        }
        else // between the bore and the face
        {
          for (l = 0; l <= Rx_max_degree; l -= 8.8)
          {
            YawRotation = l;
            PitchRotation = 0;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_checker(FK);
            Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
            transferred_point = get_Transform(registration_inv, FK);
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }
      }
      else // Any other lvl from bottom to just a lvl before the top
      {
        if (k == 0) // Creating corner bore side
        {
          YawRotation = Rx_max;
          for (l = 0; l <= RyB_max_degree; l += 5.2) // lvl one bore side yaw lowered pitch lowering
          {
            PitchRotation = l * pi / 180;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_checker(FK);
            Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
            transferred_point = get_Transform(registration_inv, FK);
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }

        else if (k == -49.0) // Creating corner face side
        {
          YawRotation = Rx_max;
          for (l = 0; l >= RyF_max_degree; l -= 7.4) // lvl one face side yaw lowered pitch increasing
          {
            PitchRotation = l * pi / 180;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_checker(FK);
            Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
            transferred_point = get_Transform(registration_inv, FK);
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }

        else // Space between two corners
        {
          YawRotation = Rx_max;
          PitchRotation = 0;
          FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
          nan_checker(FK);
          Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
          transferred_point = get_Transform(registration_inv, FK);
          points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
        }
      }
    }
  }
  YawRotation = 0;
  PitchRotation = 0;
  AxialFeetTranslation = -89;
  AxialHeadTranslation = -86;

  // Loop for creating the feet face
  //only for the bottom level at Axial Head of -86 and Axial Feet of -89
  for (k = 0.0; k <= -49.0; k -= 7.0)
  {
    LateralTranslation = k;
    // if on the bore side
    if (k >= -21)
    {
      PitchRotation = RyB_max;
      for (i = 0; i >= Rx_max_degree; i -= 8.8)
      {
        YawRotation = i;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        nan_checker(FK);
        Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
        transferred_point = get_Transform(registration_inv, FK);
        points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
      }
    }
    if (k <= -14)
    {
      PitchRotation = RyF_max;
      for (i = 0; i >= Rx_max_degree; i -= 8.8)
      {
        YawRotation = i;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        nan_checker(FK);
        Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
        transferred_point = get_Transform(registration_inv, FK);
        points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
      }
    }
  }

  YawRotation = 0;
  PitchRotation = 0;

  // Feet face for levels other than the first
  for (j = -7.1; j >= -71; j -= 7.1)
  {
    AxialHeadTranslation += j;

    for (k = 0.0; k <= -49.0; k -= 7.0)
    {
      LateralTranslation = k;
      // if on the bore side
      if (k >= -21)
      {
        PitchRotation = RyB_max;
        YawRotation = 0;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        nan_checker(FK);
        Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
        transferred_point = get_Transform(registration_inv, FK);
        points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
      }
      if (k <= -14)
      {
        PitchRotation = RyF_max;
        YawRotation = 0;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        nan_checker(FK);
        Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
        transferred_point = get_Transform(registration_inv, FK);
        points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
      }
    }
  }

  //loop for creating the sides
  AxialFeetTranslation = -3;
  AxialHeadTranslation = 0;
  LateralTranslation = 0;
  YawRotation = 0;
  PitchRotation = 0;
  double AxialHeadTranslation_old{};
  double AxialFeetTranslation_old{};
  double min_travel{-86};  // The max that the robot can move in z direction when at lowest height ( at each hight min level is changed)
  double max_travel{-157}; //157 The max that the robot can move in z direction when at highest height
  for (j = 0; j <= 71; j += 7.1)
  {
    AxialFeetTranslation += j; // For each loop it will lift the base by a constant value
    min_travel -= j;           // Takes care of the amount of Axial travel for the Axial head and feet

    AxialFeetTranslation_old = AxialFeetTranslation;
    AxialHeadTranslation_old = AxialHeadTranslation;
    for (ii = 0; ii <= min_travel; ii += (min_travel / 10)) // loop to move the base from Head to feet based on the allowable max movement range (min_travel)
    {
      AxialHeadTranslation += ii;
      AxialFeetTranslation += ii;

      for (k = 0.0; k <= -49.0; k -= 49.0)
      {
        LateralTranslation = k;
        // For the side towards bore
        if (k == 0.0)
        {
          // Conditions based on the position of the base
          // Only for the first level
          if (j == 0)
          {
            PitchRotation = RyB_max;

            // 1) Beginning of the track
            if (ii == 0)
            {
              for (i = 0; i > Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
            // 2) End of the track
            else if (ii == min_travel)
            {
              YawRotation = 0;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_checker(FK);
              Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
              transferred_point = get_Transform(registration_inv, FK);
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
            // 3) In between beginning and end
            else
            {
              for (i = 0; i > Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
          }

          // Only for the topmost level
          else if (j == 71)
          {
            for (l = RyB_max_degree / 5; l <= RyB_max_degree; l += RyB_max_degree / 5)
            {
              PitchRotation = l * pi / 180;
              // In between beginning and end
              if (ii < 0 && ii > min_travel)
              {
                for (i = Rx_max_degree / 10; i >= Rx_max_degree; i += Rx_max_degree / 10)
                {
                  YawRotation = i * pi / 180;
                  FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                          LateralTranslation, ProbeInsertion,
                                                          ProbeRotation, PitchRotation, YawRotation);
                  nan_checker(FK);
                  Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                  transferred_point = get_Transform(registration_inv, FK);
                  points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
                }
              }
            }
          }

          // For all levels between bottom and top levels
          else
          {
            PitchRotation = RyB_max;

            // 1) Beginning of the track
            if (ii == 0)
            {
              for (i = 0; i > Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }

            // 2) In between beginning and end
            else if (ii < 0 && ii > min_travel)
            {
              for (i = 0; i >= Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
          }
        }
        // For the side towards the patient
        else if (k == -49.0)
        {
          // Conditions based on the position of the base
          // Only for the first level
          if (j == 0)
          {
            PitchRotation = RyF_max;

            // 1) Beginning of the track
            if (ii == 0)
            {
              for (i = 0; i > Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
            // 2) End of the track
            else if (ii == min_travel)
            {
              YawRotation = 0;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_checker(FK);
              Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
              transferred_point = get_Transform(registration_inv, FK);
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
            // 3) In between beginning and end
            else
            {
              for (i = 0; i > Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
          }

          // Only for the topmost level
          else if (j == 71)
          {
            for (l = RyF_max_degree / 5; l >= RyF_max_degree; l += RyF_max_degree / 5)
            {
              PitchRotation = l * pi / 180;
              // In between beginning and end
              if (ii < 0 && ii > min_travel)
              {
                for (i = Rx_max_degree / 10; i >= Rx_max_degree; i += Rx_max_degree / 10)
                {
                  YawRotation = i * pi / 180;
                  FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                          LateralTranslation, ProbeInsertion,
                                                          ProbeRotation, PitchRotation, YawRotation);
                  nan_checker(FK);
                  Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                  transferred_point = get_Transform(registration_inv, FK);
                  points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
                }
              }
            }
          }

          // For all levels between bottom and top levels
          else
          {
            PitchRotation = RyF_max;

            // 1) Beginning of the track
            if (ii == 0)
            {
              for (i = 0; i > Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }

            // 2) In between beginning and end
            else if (ii < 0 && ii > min_travel)
            {
              for (i = 0; i >= Rx_max_degree; i += Rx_max_degree / 10)
              {
                YawRotation = i * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_checker(FK);
                Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);
                transferred_point = get_Transform(registration_inv, FK);
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
          }
        }
      }
      // bringing back the heads to the starting position
      AxialFeetTranslation = AxialFeetTranslation_old;
      AxialHeadTranslation = AxialHeadTranslation_old;
    }
    // resetting the location of the Axial Head and Feet to the home position
    AxialHeadTranslation = 0;
    AxialFeetTranslation = -3;
    min_travel = -86;
  }
  return points;
}

vtkSmartPointer<vtkPoints> ForwardKinematics::get_Sub_Workspace(Eigen::Matrix4d registration, Eigen::Vector4d entryPointScanner)
{
  // Create points.
  vtkSmartPointer<vtkPoints> points_RCM = vtkSmartPointer<vtkPoints>::New();
  // Object containing the 4x4 transformation matrix
  Neuro_FK_outputs FK{};
  //----------------------------------FK computation --------------------------------------------------------
  AxialHeadTranslation = 0.0;
  AxialFeetTranslation = 0.0;
  LateralTranslation = 0.0;
  PitchRotation = 0.0;
  YawRotation = 0.0;
  ProbeInsertion = 31.5;
  ProbeRotation = 0.0;
  // loop for visualizing the bottom
  double i{}, j{}, k{}, l{};                     // initializing the counters
  for (i = 0, j = 0; i < 87; i += 8.6, j += 8.6) //75
  {
    AxialFeetTranslation = i;
    AxialHeadTranslation = j;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                              LateralTranslation, ProbeInsertion,
                                              ProbeRotation, PitchRotation, YawRotation);
      points_RCM->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
    }
  }

  // Loop for visualizing the top
  for (i = 0, j = -71; i < 157; i += 6, j += 6) //75
  {
    AxialFeetTranslation = i;
    AxialHeadTranslation = j;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                              LateralTranslation, ProbeInsertion,
                                              ProbeRotation, PitchRotation, YawRotation);
      points_RCM->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
    }
  }
  const double Diff{71};
  AxialFeetTranslation = 0;
  AxialHeadTranslation = 0;

  int nan_checker_row{};
  int nan_checker_col{};
  i = 0;
  j = -1;
  k = 0;
  // Loop for creating the feet face
  for (j = -1; Diff > abs(AxialHeadTranslation - AxialFeetTranslation); --j)
  {
    AxialHeadTranslation = j;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                              LateralTranslation, ProbeInsertion,
                                              ProbeRotation, PitchRotation, YawRotation);
      points_RCM->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
    }
  }

  // Loop for creating the Head face
  AxialHeadTranslation = 86;
  AxialFeetTranslation = 86;
  nan_checker_row = 0;
  nan_checker_col = 0;
  i = 86;
  j = 86;
  k = 0;

  for (i = 87; Diff > abs(AxialHeadTranslation - AxialFeetTranslation); i += 2)
  {
    AxialFeetTranslation = i;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                              LateralTranslation, ProbeInsertion,
                                              ProbeRotation, PitchRotation, YawRotation);
      points_RCM->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
    }
  }

  //loop for creating the sides
  AxialFeetTranslation = 0;
  AxialHeadTranslation = 0;
  LateralTranslation = 0;
  nan_checker_row = 0;
  nan_checker_col = 0;
  i = 0;
  j = -1;
  k = 0;

  // double jj{};
  double min_travel{86};
  double max_travel{200};
  // double Old_AxialHeadTranslation{};
  for (j = -1; Diff > abs(j); --j)
  {
    AxialHeadTranslation = j;
    ++min_travel;

    for (ii = 0; ii <= min_travel && min_travel <= max_travel; ii += +4)
    {
      AxialHeadTranslation += 4;
      AxialFeetTranslation += 4;

      for (k = 0; k <= 37.5; k += 37.5)
      {
        LateralTranslation = k;
        FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                LateralTranslation, ProbeInsertion,
                                                ProbeRotation, PitchRotation, YawRotation);
        points_RCM->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
      }
    }
    AxialHeadTranslation = 0;
    AxialFeetTranslation = 0;
    LateralTranslation = 0;
  }

  return points_RCM;
}

// Method to search for NaN values in the FK output
void ForwardKinematics::nan_checker(Neuro_FK_outputs FK)
{
  int nan_checker_row{};
  int nan_checker_col{};
  for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
  {
    for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
    {

      if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
      {
        std::cout << "row :" << nan_checker_row << "cloumn :"
                  << "is nan!\n";
        break;
      }
    }
  }
}

Eigen::Vector4d ForwardKinematics::get_Transform(Eigen::Matrix4d registration_inv, Neuro_FK_outputs FK)
{
  //Vector that stores the End effector's position defined in the robot's Z-frame
  Eigen::Vector4d point(0.0, 0.0, 0.0, 1.0);
  for (int t = 0; t < 3; t++)
  {
    point(t) = FK.zFrameToTreatment(t, 3);
  }

  // Vector that stores the transferred points defined W.R.T the imager's frame
  Eigen::Vector4d transferred_point(0.0, 0.0, 0.0, 1.0);

  //Finding the corresponding point W.R.T the imager's frame
  transferred_point = registration_inv * point;

  //rounding step (to the tenth)
  for (int t = 0; t < 3; t++)
  {
    transferred_point(t) = round(transferred_point(t) * 10) / 10;
  }
  return transferred_point;
}
