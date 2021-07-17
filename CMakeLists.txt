cmake_minimum_required(VERSION 3.1.0)

project(mapgui VERSION 1.0)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "RELEASE")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



include_directories(
	include
    third_party/ndt_omp
	third_party/gl3w
	third_party/imgui
  	third_party/imgui/backends
  	${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

add_library(imgui STATIC
  	third_party/gl3w/gl3w.c
  	third_party/imgui/imgui.cpp
  	third_party/imgui/imgui_demo.cpp
  	third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
  	third_party/imgui/imgui_widgets.cpp
  	third_party/imgui/backends/imgui_impl_glfw.cpp
  	third_party/imgui/backends/imgui_impl_opengl3.cpp

)
target_link_libraries(imgui
  	${OPENGL_LIBRARIES}
  	glfw
  	dl
)

file(GLOB_RECURSE GLK_SRC_FILES ./src/glk/*.cpp)
file(GLOB_RECURSE GUIK_SRC_FILES ./src/guik/*.cpp)
add_library(guik STATIC
  	${GLK_SRC_FILES}
  	${GUIK_SRC_FILES}
)

file(GLOB_RECURSE HDL_SRC_FILES ./src/hdl_graph_slam/*.cpp)
add_library(hdl_graph_slam STATIC
	${HDL_SRC_FILES}
	src/g2o/robust_kernel_io.cpp
)

add_library(ndt_omp STATIC
	third_party/ndt_omp/pclomp/voxel_grid_covariance_omp.cpp
	third_party/ndt_omp/pclomp/ndt_omp.cpp
	third_party/ndt_omp/pclomp/gicp_omp.cpp
)

# target_link_libraries(hdl_graph_slam
# 	${G2O_TYPES_DATA}
#   	${G2O_CORE_LIBRARY}
#   	${G2O_STUFF_LIBRARY}
#   	${G2O_SOLVER_PCG}
#   	${G2O_SOLVER_CSPARSE}
#   	${G2O_SOLVER_CHOLMOD}
#   	${G2O_TYPES_SLAM3D}
#   	${G2O_TYPES_SLAM3D_ADDONS}
#       imgui
# )

add_executable(mapgui
  	src/main.cpp
    # ${HDL_SRC_FILES}
    src/g2o/robust_kernel_io.cpp
)

target_link_libraries(mapgui
  	${OPENGL_LIBRARIES}
  	${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  	${G2O_TYPES_DATA}
  	${G2O_CORE_LIBRARY}
  	${G2O_STUFF_LIBRARY}
  	${G2O_SOLVER_PCG}
  	${G2O_SOLVER_CSPARSE}
  	${G2O_SOLVER_CHOLMOD}
  	${G2O_TYPES_SLAM3D}
  	${G2O_TYPES_SLAM3D_ADDONS}
  	glfw
  	guik
  	imgui
    ndt_omp
  	-lboost_system
)