#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPointSource.h>
#include <vtkPoissonReconstruction.h>
#include <vtkPCANormalEstimation.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtksys/SystemTools.hxx>
#include <vtkPowerCrustSurfaceReconstruction.h>

#include "NeuroKinematics/NeuroKinematics.hpp"
#include <iostream>
#include <string>
#include <math.h> /* isnan, sqrt */
#include <cstdlib>
#include <fstream>
using std::endl;
using std::ofstream;
// namespace
// {
//   vtkSmartPointer<vtkPolyData> ReadPolyData(const char *fileName);
// }

// Initializing the joint variables for use in the FK function
double AxialHeadTranslation{0.0};
double AxialFeetTranslation{0.0};
double LateralTranslation{0.0};
double PitchRotation{0.0};
double YawRotation{0.0};
double ProbeInsertion{0.0};
double ProbeRotation{0.0};
// A is treatment to tip, where treatment is the piezoelectric element,// A = 10mm
// B is robot to entry, this allows us to specify how close to the patient the physical robot can be,// B = 5mm
// C is cannula to treatment, we define this so the robot can compute the cannula length,// C = 5mm
// D is the robot to treatment distance,// D = 41mm
// Creating an object called Forward for FK
// In the neuroRobot.cpp the specs for the  probe are: 0,0,5,41
double _cannulaToTreatment{5.0};
double _treatmentToTip{10.0};
double _robotToEntry{5.0};
double _robotToTreatmentAtHome{41.0};
Probe probe_init = {
    _cannulaToTreatment,
    _treatmentToTip,
    _robotToEntry,
    _robotToTreatmentAtHome};
// Yaw rotation range : -1.54 - +0.01
// Probe Rotation range : -6.28 - +6.28
// Pitch Rotation range : -0.46 - 0.65
// Probe Insertion range : 0.00 - 34.93
// Lateral Translation range : -49.47 - 0.00
// Axial Head Translation range : -145.01 - 0.00
// Axial Feet Translation range : -70.00 - 75.01 -> Experimental range from -7 to 233 inclusive
double i{}, j{}, k{}, l{}; //counter initialization

int nan_checker_row{};
int nan_checker_col{};
vtkSmartPointer<vtkPoints> create_RCM_workspace();
int nan_ckecker(Neuro_FK_outputs FK);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////main/////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  // Create points.
  vtkSmartPointer<vtkPoints> points =
      vtkSmartPointer<vtkPoints>::New();

  //----------------------------------FK computation --------------------------------------------------------
  NeuroKinematics Forward(&probe_init);
  Neuro_FK_outputs FK{};

  // Initializing the counters for nan output
  // Min allowed seperation 75mm
  // Max allowed seperation  146mm
  const double Diff{71}; // Is the max allowed movement while one block is stationary 146-75 = 71 mm
  const double pi{3.141};
  double Ry{};                    // Initializing the PitchRotation counter
  double RyF_max{-37 * pi / 180}; // in paper is 37.2
  double RyB_max{+30 * pi / 180}; // in paper is  30.6
  double Rx{};                    // Initializing the YawRotation counter
  double Rx_max{-90 * pi / 180};

  // Loop for visualizing the top
  for (i = 0, j = -71; i < 157; i += 15.7, j += 15.7) // 201 to 157 it should be 201 based on the paper
  {
    AxialFeetTranslation = i;
    AxialHeadTranslation = j;
    for (k = 0; k <= 37.5; k += 7.5)
    {
      LateralTranslation = k;
      FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                     LateralTranslation, ProbeInsertion,
                                     ProbeRotation, PitchRotation, YawRotation);
      nan_ckecker(FK);
      points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
          FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                         LateralTranslation, ProbeInsertion,
                                         ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
        }
      }
      else
      {
        for (Ry = 0; Ry >= -37; Ry -= 3.7)
        {
          YawRotation = Rx_max;
          PitchRotation = Ry * pi / 180;
          FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                         LateralTranslation, ProbeInsertion,
                                         ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
              FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                             LateralTranslation, ProbeInsertion,
                                             ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
              FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                             LateralTranslation, ProbeInsertion,
                                             ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
            }
          }
        }
        else // lvl one When not at the begining nor at the end (face/bore)
        {
          PitchRotation = 0;
          for (i = 0; i >= -90; i -= 9) // lvl one bore side Yaw lowering
          {
            YawRotation = i * pi / 180;
            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
          }
        }
        else if (k == 37.5) // any lvl bore side
        {
          for (i = 0; i <= 30; i += 6)
          {
            PitchRotation = i * pi / 180;
            YawRotation = 0;
            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
          }
        }

        else // any lvl bore side in between
        {
          PitchRotation = 0;
          YawRotation = 0;
          FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                         LateralTranslation, ProbeInsertion,
                                         ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
          FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                         LateralTranslation, ProbeInsertion,
                                         ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
          }
        }
        else if (k == 37.5) // any lvl bore side
        {
          for (j = 0; j <= 30; j += 1)
          {

            PitchRotation = j * pi / 180;

            FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                           LateralTranslation, ProbeInsertion,
                                           ProbeRotation, PitchRotation, YawRotation);
            nan_ckecker(FK);
            points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
          }
        }
        else // any lvl between face and bore side
        {
          PitchRotation = 0;
          FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                         LateralTranslation, ProbeInsertion,
                                         ProbeRotation, PitchRotation, YawRotation);
          nan_ckecker(FK);
          points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
                FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                               LateralTranslation, ProbeInsertion,
                                               ProbeRotation, PitchRotation, YawRotation);
                nan_ckecker(FK);
                points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
              }
            }
            else if (k == 37.5) // towards bore and top of workspace
            {
              for (Ry = 0; Ry <= 30; Ry = +6)
              {
                PitchRotation = Ry * pi / 180;
                FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                               LateralTranslation, ProbeInsertion,
                                               ProbeRotation, PitchRotation, YawRotation);
                nan_ckecker(FK);
                points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
              FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                             LateralTranslation, ProbeInsertion,
                                             ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
            }
          }
          else if (k == 37.5 && abs(j) == Diff - 1) // towards bore and top of workspace
          {
            YawRotation = 0; //Rx = 0
            for (Ry = 0; Ry <= 30; Ry += 6)
            {
              PitchRotation = Ry * pi / 180;
              FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                             LateralTranslation, ProbeInsertion,
                                             ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
            }
          }

          else if (k == 0 && abs(j) != Diff - 1) // towards face from bottom to a point before max heigth)
          {
            PitchRotation = RyF_max;
            for (Rx = 0; Rx >= -90; Rx -= 9)
            {
              YawRotation = Rx * pi / 180;
              FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                             LateralTranslation, ProbeInsertion,
                                             ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
            }
          }
          else if (k == 37.5 && abs(j) != Diff - 1) // towards bore from bottom to a point before max height
          {
            PitchRotation = RyB_max;
            for (Rx = 0; Rx >= -90; Rx -= 9)
            {
              YawRotation = Rx * pi / 180;
              FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
                                             LateralTranslation, ProbeInsertion,
                                             ProbeRotation, PitchRotation, YawRotation);
              nan_ckecker(FK);
              points->InsertNextPoint(FK.zFrameToTreatment(0, 3), FK.zFrameToTreatment(1, 3), FK.zFrameToTreatment(2, 3));
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
  if (argc == 4)
  {
    if (atoi(argv[3]) == 0)
    {
      vtkSmartPointer<vtkPoints> RCM_points = vtkSmartPointer<vtkPoints>::New();
      RCM_points = create_RCM_workspace();
      // Create a polydata object and add the points to it.
      vtkSmartPointer<vtkPolyData> polydata_RCM =
          vtkSmartPointer<vtkPolyData>::New();
      polydata_RCM->SetPoints(RCM_points);
      std::cout << "# of points: " << polydata_RCM->GetNumberOfPoints() << std::endl;
      // Write the .VTP (point cloud) file
      vtkSmartPointer<vtkXMLPolyDataWriter> writer_RCM =
          vtkSmartPointer<vtkXMLPolyDataWriter>::New();
      writer_RCM->SetFileName("RCM.vtp");
      writer_RCM->SetInputData(polydata_RCM);
      writer_RCM->Write();
      // Optional - set the mode. The default is binary.
      //writer->SetDataModeToBinary();
      //writer->SetDataModeToAscii();
      cerr << "Using PowerCrust Algorithm" << std::endl;
      vtkSmartPointer<vtkPowerCrustSurfaceReconstruction> surface_RCM =
          vtkSmartPointer<vtkPowerCrustSurfaceReconstruction>::New();
      surface_RCM->SetInputData(polydata_RCM);
      std::string filename_RCM_ply = "RCM.ply";
      vtkSmartPointer<vtkPLYWriter> plyWriter_RCM = vtkSmartPointer<vtkPLYWriter>::New();
      plyWriter_RCM->SetFileName(filename_RCM_ply.c_str());
      plyWriter_RCM->SetInputConnection(surface_RCM->GetOutputPort());
      std::cout << "Writing " << filename_RCM_ply << std::endl;
      plyWriter_RCM->Write();
    }

    // Create a polydata object and add the points to it.
    vtkSmartPointer<vtkPolyData> polydata =
        vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    std::cout << "# of points: " << polydata->GetNumberOfPoints() << std::endl;
    // Write the .VTP (point cloud) file
    vtkSmartPointer<vtkXMLPolyDataWriter> writer =
        vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName("FK.vtp");
    writer->SetInputData(polydata);
    writer->Write();
    // Optional - set the mode. The default is binary.
    //writer->SetDataModeToBinary();
    //writer->SetDataModeToAscii();
    std::string filename{"FK.ply"};

    // choose the algorithm for surface generation
    if (argc == 4 || argc == 3)
    {
      if (atoi(argv[2]) == 0)
      { // use PowerCrust algorithm
        cerr << "Using PowerCrust Algorithm" << std::endl;
        vtkSmartPointer<vtkPowerCrustSurfaceReconstruction> surface =
            vtkSmartPointer<vtkPowerCrustSurfaceReconstruction>::New();
        surface->SetInputData(polydata);
        std::string filename = argv[1];
        vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
        plyWriter->SetFileName(filename.c_str());
        plyWriter->SetInputConnection(surface->GetOutputPort());
        std::cout << "Writing " << filename << std::endl;
        plyWriter->Write();
        return EXIT_SUCCESS;
      }
      else if (atoi(argv[2]) == 1)
      { // creating a surface using Poisson's algorithm

        cerr << "Using Poisson's Algorithm" << std::endl;
        vtkSmartPointer<vtkPoissonReconstruction> surface =
            vtkSmartPointer<vtkPoissonReconstruction>::New();
        surface->SetDepth(12);
        int sampleSize = polydata->GetNumberOfPoints() * .00005;
        if (sampleSize < 10)
        {
          sampleSize = 10;
        }
        if (polydata->GetPointData()->GetNormals())
        {
          std::cout << "Using normals from input file" << std::endl;
          surface->SetInputData(polydata);
        }
        else
        {
          std::cout << "Estimating normals using PCANormalEstimation" << std::endl;
          vtkSmartPointer<vtkPCANormalEstimation> normals =
              vtkSmartPointer<vtkPCANormalEstimation>::New();
          normals->SetInputData(polydata);
          normals->SetSampleSize(sampleSize);
          normals->SetNormalOrientationToGraphTraversal();
          normals->FlipNormalsOff();
          surface->SetInputConnection(normals->GetOutputPort());
        }
        std::string filename = argv[1];
        vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
        plyWriter->SetFileName(filename.c_str());
        plyWriter->SetInputConnection(surface->GetOutputPort());
        std::cout << "Writing " << filename << std::endl;
        plyWriter->Write();
        return EXIT_SUCCESS;
      }
      else
      {
        cerr << " Usage : " << std::endl;
        cerr << " first argument : name.ply" << std::endl;
        cerr << " second argument : 0 for PowerCrust Algorithm, 1 for Poisson's algorithm" << std::endl;
        EXIT_FAILURE;
      }
    }
    else
    {
      cerr << " Usage : " << std::endl;
      cerr << " first argument : <name.ply>" << std::endl;
      cerr << " second argument : <0> for PowerCrust Algorithm, <1> for Poisson's algorithm" << std::endl;
      EXIT_FAILURE;
    }
  }
}
// Method to search for nan values in the FK output
int nan_ckecker(Neuro_FK_outputs FK)
{
  for (nan_checker_row = 0; nan_checker_row < 4; ++nan_checker_row) // Loop for checking NaN
  {
    for (nan_checker_col = 0; nan_checker_col < 4; ++nan_checker_col)
    {

      if (isnan(FK.zFrameToTreatment(nan_checker_row, nan_checker_col)))
      {
        std::cout << "row :" << nan_checker_row << "cloumn :"
                  << "is nan!\n";
        return 1;
        break;
      }
    }
  }
  return 0;
};

// Method to create the RCM workspace used to visualize the general workspace in Slicer
vtkSmartPointer<vtkPoints> create_RCM_workspace()
{
  // Create points.
  vtkSmartPointer<vtkPoints> points_RCM =
      vtkSmartPointer<vtkPoints>::New();

  //----------------------------------FK computation --------------------------------------------------------
  NeuroKinematics Forward(&probe_init);
  Neuro_FK_outputs FK{};
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
      FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
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
      FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
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
      FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
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
      FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
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
        FK = Forward.ForwardKinematics(AxialHeadTranslation, AxialFeetTranslation,
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
