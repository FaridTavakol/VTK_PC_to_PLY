
cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(PLY_creator)

include_directories(NeuroKinematics NeuroKinematics/)

add_library(NeuroKinematics NeuroKinematics/NeuroKinematics.cpp)

find_package(VTK COMPONENTS
  vtkCommonColor
  vtkCommonCore
  vtkCommonDataModel
  vtkFiltersPoints
  vtkFiltersSources
  vtkIOGeometry
  vtkIOLegacy
  vtkIOPLY
  vtkIOXML
  vtkInteractionStyle
  vtkPoissonReconstruction
  vtkPowercrust
  vtkRenderingContextOpenGL2
  vtkRenderingCore
  vtkRenderingFreeType
  vtkRenderingGL2PSOpenGL2
  vtkRenderingOpenGL2 QUIET
)
  
if (NOT VTK_FOUND)
  message("Skipping test_ply: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  # old system
  include(${VTK_USE_FILE})
  add_executable(ForwardKinematics MACOSX_BUNDLE ForwardKinematics.cpp )
  target_link_libraries(ForwardKinematics PRIVATE ${VTK_LIBRARIES})
else ()
  # include all components
  add_executable(ForwardKinematics MACOSX_BUNDLE ForwardKinematics.cpp )
  target_link_libraries(ForwardKinematics PRIVATE ${VTK_LIBRARIES} NeuroKinematics)
  # vtk_module_autoinit is needed
  vtk_module_autoinit(
    TARGETS ForwardKinematics
    MODULES ${VTK_LIBRARIES})
endif () 
