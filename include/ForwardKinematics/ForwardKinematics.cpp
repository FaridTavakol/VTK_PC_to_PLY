// #include <vtkCellArray.h>
// #include <vtkPoints.h>
// #include <vtkSmartPointer.h>
// #include <vtkPointSource.h>
// #include <vtkPointData.h>
// #include <vtkProperty.h>
// #include <vtksys/SystemTools.hxx>
// #include <iostream>
// #include <string>
// #include <math.h> /* isnan, sqrt */
// #include <cstdlib>
#include "ForwardKinematics.h"
using std::endl;
using std::ofstream;

// A is treatment to tip, where treatment is the piezoelectric element,// A = 10mm
// B is robot to entry, this allows us to specify how close to the patient the physical robot can be,// B = 5mm
// C is cannula to treatment, we define this so the robot can compute the cannula length,// C = 5mm
// D is the robot to treatment distance,// D = 41mm
// Creating an object called Forward for FK
// In the neuroRobot.cpp the specs for the  probe are: 0,0,5,41

ForwardKinematics::ForwardKinematics(NeuroKinematics &NeuroKinematics) : Diff(71), pi(3.141)
{
  //counters
  i = 0;
  j = 0;
  k = 0;
  l = 0; //counter initialization
  // Min allowed seperation 75mm
  // Max allowed seperatio1f46mm
  Ry = 0.0;                 // Initializing the PitchRotation counter
  RyF_max = -37 * pi / 180; // in paper is 37.2
  RyB_max = +30 * pi / 180; // in paper is  30.6
  Rx = 0.0;                 // Initializing the YawRotation counter
  Rx_max = -90 * pi / 180;  // Max YawRotation

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
  // To visualize the transferred points in the slicer
  Eigen::Matrix4d registration_inv = registration.inverse();
  std::cerr << registration_inv << std::endl;
  Eigen::Vector4d transferred_point{};
  Eigen::Vector4d point{};

  transferred_point(3) = 1;
  std::cerr << transferred_point << std::endl;
  for (int tt = 0; tt < 3; tt++)
  {
    transferred_point(tt) = round(transferred_point(tt) * 10) / 10;
  }
  std::cerr << transferred_point << std::endl;

  // Object containing the 4x4 transformation matrix
  Neuro_FK_outputs FK{};

  // Create points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  //----------------------------------FK computation --------------------------------------------------------
  // Loop for visualizing the top
  for (i = 0, j = -71; i < 157; i += 15.7, j += 15.7) // 201 to 157 it should be 201 based on the paper
  {
    AxialFeetTranslation = i;
    AxialHeadTranslation = j;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                              LateralTranslation, ProbeInsertion,
                                              ProbeRotation, PitchRotation, YawRotation);
      nan_ckecker(FK);
      for (int t = 0; t < 3; t++)
      {
        transferred_point(t) = FK.zFrameToTreatment(t, 3);
      }
      transferred_point = registration_inv * point;
      //rounding step
      for (int t = 0; t < 3; t++)
      {
        transferred_point(t) = round(transferred_point(t) * 10) / 10;
      }
      points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
    }
  }

  // loop for visualizing the bottom
  for (i = 0, j = 0; i < 87; i += 8.7, j += 8.7) //75
  {
    AxialFeetTranslation = i;
    AxialHeadTranslation = j;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      if (k == 37.5)
      {
        for (Ry = 0; Ry <= 30; Ry += 5)
        {
          YawRotation = Rx_max;
          PitchRotation = Ry * pi / 180;
          ;
          FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = FK.zFrameToTreatment(t, 3);
          }
          transferred_point = registration_inv * point;
          //rounding step
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = round(transferred_point(t) * 10) / 10;
          }
          points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
        }
      }
      else
      {
        for (Ry = 0; Ry >= -37; Ry -= 3.7)
        {
          YawRotation = Rx_max;
          PitchRotation = Ry * pi / 180;
          FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = FK.zFrameToTreatment(t, 3);
          }
          transferred_point = registration_inv * point;
          //rounding step
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = round(transferred_point(t) * 10) / 10;
          }
          points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
        }
      }
    }
  }

  YawRotation = 0;
  PitchRotation = 0;
  AxialFeetTranslation = 0;
  AxialHeadTranslation = 0;

  i = 0;
  j = -1;
  k = 0;
  // Loop for creating the feet face
  for (j = -1; abs(AxialHeadTranslation - AxialFeetTranslation) < Diff; j -= 5)
  {
    AxialHeadTranslation = j;

    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      // only for the first lvl
      if (j == -1) //lvl one    && -37.5
      {
        if (k == 0) // lvl one face side
        {
          for (i = 0; i >= -90; i -= 9) // lvl one face side Yaw lowering
          {
            YawRotation = i * pi / 180;
            for (l = 0; l >= -37.5; l -= 7.5) // lvl one face side yaw lowered pitch lowering
            {
              PitchRotation = l * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = FK.zFrameToTreatment(t, 3);
              }
              transferred_point = registration_inv * point;
              //rounding step
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = round(transferred_point(t) * 10) / 10;
              }
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
        }

        else if (k == 37.5) // lvl one bore side
        {
          for (i = 0; i >= -90; i -= 9) // lvl one bore side Yaw lowering
          {
            YawRotation = i * pi / 180;
            for (l = 0; l <= 30; l += 6) // lvl one face side yaw lowered pitch increasing
            {
              PitchRotation = l * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = FK.zFrameToTreatment(t, 3);
              }
              transferred_point = registration_inv * point;
              //rounding step
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = round(transferred_point(t) * 10) / 10;
              }
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
        }
        else // lvl one When not at the begining nor at the end (face/bore)
        {
          PitchRotation = 0;
          for (i = 0; i >= -90; i -= 9) // lvl one bore side Yaw lowering
          {
            YawRotation = i * pi / 180;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = FK.zFrameToTreatment(t, 3);
            }
            transferred_point = registration_inv * point;
            //rounding step
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = round(transferred_point(t) * 10) / 10;
            }
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }
      }
      //end of first lvl

      else
      {
        if (k == 0) // any lvl face side
        {
          for (i = 0; i >= -37.5; i -= 7.5)
          {
            PitchRotation = i * pi / 180;
            YawRotation = 0;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = FK.zFrameToTreatment(t, 3);
            }
            transferred_point = registration_inv * point;
            //rounding step
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = round(transferred_point(t) * 10) / 10;
            }
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }
        else if (k == 37.5) // any lvl bore side
        {
          for (i = 0; i <= 30; i += 6)
          {
            PitchRotation = i * pi / 180;
            YawRotation = 0;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = FK.zFrameToTreatment(t, 3);
            }
            transferred_point = registration_inv * point;
            //rounding step
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = round(transferred_point(t) * 10) / 10;
            }
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }

        else // any lvl bore side in between
        {
          PitchRotation = 0;
          YawRotation = 0;
          FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = FK.zFrameToTreatment(t, 3);
          }
          transferred_point = registration_inv * point;
          //rounding step
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = round(transferred_point(t) * 10) / 10;
          }
          points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
        }
      }
    }
  }

  // Loop for creating the Head face
  // j = 200 i = 200 i++ 71 = 271 ;
  PitchRotation = 0;
  YawRotation = 0;

  AxialHeadTranslation = 86;
  AxialFeetTranslation = 86;
  i = 86;
  j = 86;
  k = 0;
  for (i = 87; abs(AxialHeadTranslation - AxialFeetTranslation) <= Diff; ++i)
  {
    AxialFeetTranslation = i;

    for (k = 0; k <= 37.5; k += 3.75)
    {
      LateralTranslation = k;

      if (abs(AxialHeadTranslation - AxialFeetTranslation) == Diff) // highest lvl
      {
        for (Rx = 0; Rx >= -90; Rx -= 9)
        {
          PitchRotation = 0;
          YawRotation = Rx * pi / 180;
          FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = FK.zFrameToTreatment(t, 3);
          }
          transferred_point = registration_inv * point;
          //rounding step
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = round(transferred_point(t) * 10) / 10;
          }
          points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
        }
      }
      else // all lvls before highest lvl
      {
        YawRotation = Rx_max;

        if (k == 0) // any lvl face side
        {
          for (j = 0; j >= -37.5; j -= 0.5)
          {
            PitchRotation = j * pi / 180;
            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = FK.zFrameToTreatment(t, 3);
            }
            transferred_point = registration_inv * point;
            //rounding step
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = round(transferred_point(t) * 10) / 10;
            }
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }
        else if (k == 37.5) // any lvl bore side
        {
          for (j = 0; j <= 30; j += 1)
          {

            PitchRotation = j * pi / 180;

            FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                    LateralTranslation, ProbeInsertion,
                                                    ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = FK.zFrameToTreatment(t, 3);
            }
            transferred_point = registration_inv * point;
            //rounding step
            for (int t = 0; t < 3; t++)
            {
              transferred_point(t) = round(transferred_point(t) * 10) / 10;
            }
            points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
          }
        }
        else // any lvl between face and bore side
        {
          PitchRotation = 0;
          FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                  LateralTranslation, ProbeInsertion,
                                                  ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = FK.zFrameToTreatment(t, 3);
          }
          transferred_point = registration_inv * point;
          //rounding step
          for (int t = 0; t < 3; t++)
          {
            transferred_point(t) = round(transferred_point(t) * 10) / 10;
          }
          points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
        }
      }
    }
  }
  YawRotation = 0; //resetting the Yaw value to 0

  //loop for creating the sides
  AxialFeetTranslation = 0;
  AxialHeadTranslation = 0;
  LateralTranslation = 0;
  i = 0;
  j = -1;
  k = 0;
  double ii{};
  // double jj{};
  double min_travel{86};  // The max that the robot can move in z direction when at lowest heigth ( at each hight min level is changed)
  double max_travel{156}; // The max that the robot can move in z direction when at highest heigth
  for (j = -1; Diff > abs(j); j -= 5)
  {
    AxialHeadTranslation = j;
    ++min_travel;

    for (ii = 0; ii <= min_travel && min_travel <= max_travel; ii += 5)
    {
      ++AxialHeadTranslation;
      ++AxialFeetTranslation;

      for (k = 0; k <= 37.5; k += 37.5) // Chooses sides for generation of the PC
      {
        LateralTranslation = k;

        if (min_travel == max_travel && ii == min_travel) // when head is at the highest and to the last point towards feet
        {
          for (Rx = 0; Rx >= -90; Rx -= 9)
          {
            YawRotation = Rx * pi / 180;

            if (k == 0) // towards face and top of workspace
            {
              for (Ry = 0; Ry >= -37; Ry -= 9)
              {
                PitchRotation = Ry * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_ckecker(FK);
                for (int t = 0; t < 3; t++)
                {
                  transferred_point(t) = FK.zFrameToTreatment(t, 3);
                }
                transferred_point = registration_inv * point;
                //rounding step
                for (int t = 0; t < 3; t++)
                {
                  transferred_point(t) = round(transferred_point(t) * 10) / 10;
                }
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
            else if (k == 37.5) // towards bore and top of workspace
            {
              for (Ry = 0; Ry <= 30; Ry = +6)
              {
                PitchRotation = Ry * pi / 180;
                FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                        LateralTranslation, ProbeInsertion,
                                                        ProbeRotation, PitchRotation, YawRotation);
                nan_ckecker(FK);
                for (int t = 0; t < 3; t++)
                {
                  transferred_point(t) = FK.zFrameToTreatment(t, 3);
                }
                transferred_point = registration_inv * point;
                //rounding step
                for (int t = 0; t < 3; t++)
                {
                  transferred_point(t) = round(transferred_point(t) * 10) / 10;
                }
                points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
              }
            }
          }
        }
        // Other cases other than the above
        else
        {
          if (k == 0 && abs(j) == Diff - 1) // towards face and top of workspace
          {
            YawRotation = 0; //Rx = 0
            for (Ry = 0; Ry >= -37; Ry -= 3.7)
            {
              PitchRotation = Ry * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = FK.zFrameToTreatment(t, 3);
              }
              transferred_point = registration_inv * point;
              //rounding step
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = round(transferred_point(t) * 10) / 10;
              }
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
          else if (k == 37.5 && abs(j) == Diff - 1) // towards bore and top of workspace
          {
            YawRotation = 0; //Rx = 0
            for (Ry = 0; Ry <= 30; Ry += 6)
            {
              PitchRotation = Ry * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = FK.zFrameToTreatment(t, 3);
              }
              transferred_point = registration_inv * point;
              //rounding step
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = round(transferred_point(t) * 10) / 10;
              }
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }

          else if (k == 0 && abs(j) != Diff - 1) // towards face from bottom to a point before max heigth)
          {
            PitchRotation = RyF_max;
            for (Rx = 0; Rx >= -90; Rx -= 9)
            {
              YawRotation = Rx * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = FK.zFrameToTreatment(t, 3);
              }
              transferred_point = registration_inv * point;
              //rounding step
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = round(transferred_point(t) * 10) / 10;
              }
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
          else if (k == 37.5 && abs(j) != Diff - 1) // towards bore from bottom to a point before max height
          {
            PitchRotation = RyB_max;
            for (Rx = 0; Rx >= -90; Rx -= 9)
            {
              YawRotation = Rx * pi / 180;
              FK = NeuroKinematics_.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                                      LateralTranslation, ProbeInsertion,
                                                      ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = FK.zFrameToTreatment(t, 3);
              }
              transferred_point = registration_inv * point;
              //rounding step
              for (int t = 0; t < 3; t++)
              {
                transferred_point(t) = round(transferred_point(t) * 10) / 10;
              }
              points->InsertNextPoint(transferred_point(0), transferred_point(1), transferred_point(2));
            }
          }
        }
        YawRotation = 0;
        PitchRotation = 0;
      }
      LateralTranslation = 0;
      // if (ii > 150)
      //   std::cout << "valuse of ii " << ii << std::endl;
      // if (min_travel == 156)
      //   std::cout << "valuse :" << ii << std::endl;
    }
    AxialHeadTranslation = 0;
    AxialFeetTranslation = 0;
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
  double ii{};
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
void ForwardKinematics::nan_ckecker(Neuro_FK_outputs FK)
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
};
