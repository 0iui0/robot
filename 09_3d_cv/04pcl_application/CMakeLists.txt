cmake_minimum_required(VERSION 3.10)
project(pcl_application)
set(CMAKE_CXX_STANDARD 14)

# =========================
# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)
# =========================

set(PCL_DIR ~/miniconda3/envs/ros_env/share/pcl-1.12)
find_package(PCL)

#add_executable(01planar_segmentation 01pc_segmentation/01planar_segmentation.cpp)
#target_link_libraries(01planar_segmentation ${PCL_LIBRARIES})

#add_executable(02_cylinder_segementation 01pc_segmentation/02_cylinder_segementation.cpp)
#target_link_libraries(02_cylinder_segementation ${PCL_LIBRARIES})

#add_executable(03_cluster_extraction 01pc_segmentation/03_cluster_extraction.cpp)
#target_link_libraries(03_cluster_extraction ${PCL_LIBRARIES})

#add_executable(02resampling 02resampling.cpp)
#target_link_libraries(02resampling ${PCL_LIBRARIES})

add_executable(03moment_of_inertia bounding_box.cpp 03moment_of_inertia.cpp )
target_link_libraries(03moment_of_inertia ${PCL_LIBRARIES})

# add_executable(04pc_template_match 04pc_template_match.cpp)
# target_link_libraries(04pc_template_match ${PCL_LIBRARIES})

add_executable(narf_keypoint_extraction  narf_keypoint_extraction.cpp )
target_link_libraries(narf_keypoint_extraction ${PCL_LIBRARIES})

add_executable(harris3d_keypoint_extraction  harris3d_keypoint_extraction.cpp )
target_link_libraries(harris3d_keypoint_extraction ${PCL_LIBRARIES})

add_executable(iss_keypoint_extraction  iss_keypoint_extraction.cpp )
target_link_libraries(iss_keypoint_extraction ${PCL_LIBRARIES})
