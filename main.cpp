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
namespace
{
    vtkSmartPointer<vtkPolyData> ReadPolyData(const char *fileName);
}

int main(int argc, char *argv[])
{
    // Create points.
    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();

    for (float i = 0; i <= 200; i += 2)
    {
        for (float j = 0; j <= 200; j += 2)
        {
            for (float k = 0; k <= 200; k += 200)
            {
                points->InsertNextPoint(i, j, k);
            }
        }
    }
    for (float j = 0; j <= 200; j += 2)
    {
        for (float k = 0; k <= 200; k += 2)
        {
            for (float i = 0; i <= 200; i += 200)
            {
                points->InsertNextPoint(i, j, k);
            }
        }
    }
    for (float k = 0; k <= 200; k += 2)
    {
        for (float i = 0; i <= 200; i += 2)
        {
            for (float j = 0; j <= 200; j += 200)
            {
                points->InsertNextPoint(i, j, k);
            }
        }
    }
    // Create a polydata object and add the points to it.
    vtkSmartPointer<vtkPolyData> polydata =
        vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    std::cout << "# of points: " << polydata->GetNumberOfPoints() << std::endl;
    // creating a surface using Poisson's algorithm
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

    // // Write the file
    // vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    //     vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    // writer->SetFileName("test.vtp");
    // writer->SetInputData(polydata);

    // Optional - set the mode. The default is binary.
    //writer->SetDataModeToBinary();
    //writer->SetDataModeToAscii();

    // writer->Write();

    ///////////////////////////////////////////////////////////////////////
    std::string filename = argv[1];
    vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
    plyWriter->SetFileName(filename.c_str());
    plyWriter->SetInputConnection(surface->GetOutputPort());
    plyWriter->Write();

    return EXIT_SUCCESS;
}
