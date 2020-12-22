#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkPLYWriter.h>
#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkPoissonReconstruction.h>
#include <vtkPCANormalEstimation.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtksys/SystemTools.hxx>
#include <vtkPowerCrustSurfaceReconstruction.h>
#include "NeuroKinematics/NeuroKinematics.hpp"
#include <iostream>
#include <string>
#include <math.h> /* isnan, sqrt */
#include <cstdlib>
#include "include/ForwardKinematics/ForwardKinematics.h"

double _cannulaToTreatment{5.0};
double _treatmentToTip{10.0};
double _robotToEntry{5.0};
double _robotToTreatmentAtHome{41.0};
Probe probe_init = {
    _cannulaToTreatment,
    _treatmentToTip,
    _robotToEntry,
    _robotToTreatmentAtHome};

int main(int argc, char *argv[])
{
    NeuroKinematics NeuroKinematics_(&probe_init);
    ForwardKinematics ForwardKinematics_(NeuroKinematics_);
    // General workspace computation
    // 4x4 Registration matrix
    // just a test case, use X= -53, Y= -119, Z= -121
    Eigen::Matrix4d registration;
    registration << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Generating General Workspace
    vtkSmartPointer<vtkPoints> General_Workspace_PC = vtkSmartPointer<vtkPoints>::New();
    General_Workspace_PC = ForwardKinematics_.get_General_Workspace(registration, General_Workspace_PC);
    // std::cout << "# of points: " << General_Workspace_PC->GetNumberOfPoints();
    // vtkSmartPointer<vtkPoints> RCM_points = vtkSmartPointer<vtkPoints>::New();
    // Create a polydata object and add the points to it.
    vtkSmartPointer<vtkPolyData> polydata_General_Workspace_PC = vtkSmartPointer<vtkPolyData>::New();
    polydata_General_Workspace_PC->SetPoints(General_Workspace_PC);
    std::cout << "# of points: " << polydata_General_Workspace_PC->GetNumberOfPoints() << std::endl;

    // Write the .VTP (point cloud) file
    vtkSmartPointer<vtkXMLPolyDataWriter> writer_General_Workspace_PC =
        vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer_General_Workspace_PC->SetFileName("General_Workspace.vtp");
    writer_General_Workspace_PC->SetInputData(polydata_General_Workspace_PC);
    writer_General_Workspace_PC->Write();

    std::cerr << "Using PowerCrust Algorithm to create General Workspace surface mesh" << std::endl;
    vtkSmartPointer<vtkPowerCrustSurfaceReconstruction> surface_General_Workspace =
        vtkSmartPointer<vtkPowerCrustSurfaceReconstruction>::New();
    surface_General_Workspace->SetInputData(polydata_General_Workspace_PC);
    std::string filename_General_workspace_ply = "General_Workspace.ply";
    vtkSmartPointer<vtkPLYWriter> plyWriter_General_Workspace = vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter_General_Workspace->SetFileName(filename_General_workspace_ply.c_str());
    plyWriter_General_Workspace->SetInputConnection(surface_General_Workspace->GetOutputPort());
    std::cout << "Writing " << filename_General_workspace_ply << std::endl;
    plyWriter_General_Workspace->Write();

    // Generating RCM Workspace
    vtkSmartPointer<vtkPoints> RCM_Workspace_PC = vtkSmartPointer<vtkPoints>::New();
    RCM_Workspace_PC = ForwardKinematics_.get_RCM_Workspace(registration, RCM_Workspace_PC);
    // std::cout << "# of points: " << RCM_Workspace_PC->GetNumberOfPoints();
    // Create a polydata object and add the points to it.
    vtkSmartPointer<vtkPolyData> polydata_RCM_Workspace_PC = vtkSmartPointer<vtkPolyData>::New();
    polydata_RCM_Workspace_PC->SetPoints(RCM_Workspace_PC);
    std::cout << "# of points: " << polydata_RCM_Workspace_PC->GetNumberOfPoints() << std::endl;

    // Write the .VTP (point cloud) file
    vtkSmartPointer<vtkXMLPolyDataWriter> writer_RCM_Workspace_PC =
        vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer_RCM_Workspace_PC->SetFileName("General_Workspace.vtp");
    writer_RCM_Workspace_PC->SetInputData(polydata_RCM_Workspace_PC);
    writer_RCM_Workspace_PC->Write();

    std::cerr << "Using PowerCrust Algorithm to create RCM Workspace surface mesh" << std::endl;
    vtkSmartPointer<vtkPowerCrustSurfaceReconstruction> surface_RCM_Workspace =
        vtkSmartPointer<vtkPowerCrustSurfaceReconstruction>::New();
    surface_RCM_Workspace->SetInputData(polydata_RCM_Workspace_PC);
    std::string filename_RCM_workspace_ply = "RCM_Workspace.ply";
    vtkSmartPointer<vtkPLYWriter> plyWriter_RCM_Workspace = vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter_RCM_Workspace->SetFileName(filename_RCM_workspace_ply.c_str());
    plyWriter_RCM_Workspace->SetInputConnection(surface_RCM_Workspace->GetOutputPort());
    std::cout << "Writing " << filename_RCM_workspace_ply << std::endl;
    plyWriter_RCM_Workspace->Write();

    // Testing the RCM_PC method
    ForwardKinematics_.get_RCM_PC(registration);
}