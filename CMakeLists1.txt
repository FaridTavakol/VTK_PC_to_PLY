# cmake_minimum_required(VERSION 3.0.0)

# add_definitions() #-DDEBUG -DVIEW_TABLE

# project(Kinematics)

# # find_package (Eigen3 3.3 REQUIRED)

# include_directories(NeuroKinematics NeuroKinematics/)

# add_library(NeuroKinematics NeuroKinematics/NeuroKinematics.cpp)

# find_package (Python2 COMPONENTS Development NumPy)#added

# add_executable(ForwardKinematics ForwardKinematics.cpp)


# target_include_directories(ForwardKinematics PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS}) #added

# target_link_libraries(ForwardKinematics
#   # Eigen3,
#   NeuroKinematics
#  Python2::Python Python2::NumPy)

# find_package(Python2 COMPONENTS Development NumPy)
# target_include_directories(myproject PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
# target_link_libraries(myproject Python2::Python Python2::NumPy)


# # Inverse Kinematics
# cmake_minimum_required(VERSION 3.0.0)

# add_definitions() #-DDEBUG -DVIEW_TABLE

# project(Inverse)

# include_directories(NeuroKinematics NeuroKinematics/)

# add_library(NeuroKinematics NeuroKinematics/NeuroKinematics.cpp)
# add_executable(InverseKinematics InverseKinematics.cpp)
# target_link_libraries(InverseKinematics
#   NeuroKinematics
# )
# ==========================IK_Solver================================
# Inverse Kinematics
cmake_minimum_required(VERSION 3.0.0)

add_definitions() #-DDEBUG -DVIEW_TABLE

project(IK_Solver)

include_directories(NeuroKinematics NeuroKinematics/)

add_library(NeuroKinematics NeuroKinematics/NeuroKinematics.cpp)
add_executable(IK_Solver IK_Solver.cpp)
target_link_libraries(IK_Solver
  NeuroKinematics
)
