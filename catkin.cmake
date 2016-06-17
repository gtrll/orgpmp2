cmake_minimum_required(VERSION 2.8.3)

# catkin
find_package(catkin REQUIRED COMPONENTS openrave_catkin)
catkin_package()
catkin_python_setup()

# C++11 flags for GTSAM 4.0
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# find OpenRAVE
find_package(OpenRAVE REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

# find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)

# find GPMP2
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})
set(gpmp2_LIBRARIES gpmp2)

openrave_plugin("${PROJECT_NAME}_plugin"
    src/orgpmp2.cpp
    src/orgpmp2_mod.cpp
    src/orgpmp2_kdata.cpp
    src/orcwrap.cpp
    src/utils/grid.c
    src/utils/grid_flood.c
    src/utils/kin.c
    src/utils/mat.c
    src/utils/util_shparse.c
)
target_link_libraries("${PROJECT_NAME}_plugin"
    blas
    lapacke
    lapack
    gsl
    ${GTSAM_LIBRARIES}
    ${gpmp2_LIBRARIES}
    ${catkin_LIBRARIES}
)