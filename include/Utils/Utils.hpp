#pragma once
#include "../../NeuroKinematics/NeuroKinematics.hpp"
#include <fstream>
#include "/usr/include/vtk-6.3/vtkPoints.h"
#include "/usr/include/vtk-6.3/vtkSmartPointer.h"
using std::ofstream;

class Utils
{ // constructor
    Utils();
    // Methods
public:
    void save_To_XYZ(Eigen::Matrix3Xf Point_Cloud, const char *file_name);
    void save_To_XYZ(Eigen::Matrix3Xf Point_Cloud, const std::string &file_name);
    vtkSmartPointer<vtkPoints> save_To_VTK(Eigen::Matrix3Xf Point_Cloud);
    // vtkSmartPointer<vtkPoints> save_To_VTK(Eigen::Matrix3Xf Point_Cloud);
};