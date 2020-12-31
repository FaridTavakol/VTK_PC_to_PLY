#include "Utils.hpp"

Utils::Utils()
{
}

// Method that takes an Eigen matrix and converts it into an xyz point cloud format
void Utils::save_To_XYZ(Eigen::Matrix3Xf Point_Cloud, const char *file_name)
{
    ofstream file(file_name);
    int no_cols = Point_Cloud.cols(); // number of points in the points cloud
    for (int i = 0; i < no_cols; i++)
    {
        file << Point_Cloud(0, i) << " " << Point_Cloud(1, i) << " " << Point_Cloud(2, i) << " 0.00 0.00 0.00" << std::endl;
    }
    file.close();
}
void Utils::save_To_XYZ(Eigen::Matrix3Xf Point_Cloud, const std::string &file_name)
{
    ofstream file(file_name);
    int no_cols = Point_Cloud.cols(); // number of points in the points cloud
    for (int i = 0; i < no_cols; i++)
    {
        file << Point_Cloud(0, i) << " " << Point_Cloud(1, i) << " " << Point_Cloud(2, i) << " 0.00 0.00 0.00" << std::endl;
    }
    file.close();
}
vtkSmartPointer<vtkPoints> Utils::save_To_VTK(Eigen::Matrix3Xf Point_Cloud)
{
    vtkSmartPointer<vtkPoints> point = vtkSmartPointer<vtkPoints>::New();

    int no_cols = Point_Cloud.cols(); // number of points in the points cloud
    for (int i = 0; i < no_cols; i++)
    {
        point->InsertNextPoint(Point_Cloud(0, i), Point_Cloud(1, i), Point_Cloud(2, i));
    }
    return point;
}
