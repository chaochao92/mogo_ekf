cmake_minimum_required( VERSION 3.0 )
project( ekf )

if( ${PROJECT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR} )
  #set( CMAKE_CXX_COMPILER "g++" )
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set( CMAKE_BUILD_TYPE "Release" )
endif()

if( NOT DEFINED MODULE_OUT_PATH )
  set( MODULE_OUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib/ )
endif()

set( LIBRARY_OUTPUT_PATH ${MODULE_OUT_PATH} )

set( TARGET_BUILD_FLAGS
  -Wall
  -pthread
  $<$<NOT:$<BOOL:${CUSTOM_FLAG_S32V}>>:-msse3>
  $<$<NOT:$<BOOL:${CUSTOM_FLAG_S32V}>>:-march=native>
  "$<$<CONFIG:Release>:-O3>"
  "$<$<NOT:$<CONFIG:Release>>:-O0>"
  "$<$<NOT:$<CONFIG:Release>>:-ggdb>"
)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        )

find_package( Threads )

find_package( Eigen3 3.3 REQUIRED NO_MODULE )
IF(Eigen3_FOUND)
    message("Eigen3 lib is found: " ${EIGEN3_INCLUDE_DIR})
    #include_directories(${EIGEN3_INCLUDE_DIR}) #DO NOT USE THIS GLOBAL OPTION
ENDIF(Eigen3_FOUND)

# Project include

catkin_package(
          INCLUDE_DIRS include
          CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${PROJECT_SOURCE_DIR}/include 	# for public access
        ${PROJECT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# ============== EKF utilities library ===============
# *** This is a commonly used header-based library   *** 
# ======================================================
set( EKF_UTILS_LIB "${PROJECT_NAME}_utils" )
add_library(${EKF_UTILS_LIB} STATIC
  utils/print_utils.hpp
  utils/quat_utils.hpp
  utils/vec_utils.hpp
  utils/coord_utils.hpp
)
target_compile_features( ${EKF_UTILS_LIB} PUBLIC cxx_std_14 )
target_compile_options( ${EKF_UTILS_LIB}  PUBLIC
    ${TARGET_BUILD_FLAGS}
)
set_target_properties( ${EKF_UTILS_LIB}  PROPERTIES
    LINKER_LANGUAGE CXX			# Because .hpp implementation
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
) 
target_include_directories( ${EKF_UTILS_LIB}  PUBLIC
    ${PROJECT_SOURCE_DIR}/utils 		# Exposed header directory
)
target_link_libraries( ${EKF_UTILS_LIB} 
  ${CMAKE_THREAD_LIBS_INIT}
  Eigen3::Eigen
)
# ======================================================

function(COMMON_COMPILE_PROP NAME_ARG)
  target_compile_features( ${NAME_ARG} PUBLIC cxx_std_14 )
  target_compile_options( ${NAME_ARG}
    PUBLIC
      ${TARGET_BUILD_FLAGS}
  )
  set_target_properties( ${NAME_ARG}
    PROPERTIES
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
  )
endfunction()



# ===== EKF core library ===== 
set(EKF_CORE_LIB "${PROJECT_NAME}_core" )
add_library(${EKF_CORE_LIB} STATIC
  src/ekf.h
  src/ekf_calc_fd.cpp  
  src/ekf_calc_qd.cpp  
  src/ekf_data_loader.cpp
  src/ekf_initialization.cpp
  src/ekf_update.cpp
  src/ekf_predict.cpp 
  src/ekf_run.cpp  
  src/ekf_state_evaluation.cpp
)
common_compile_prop( ${EKF_CORE_LIB} )
target_include_directories( ${EKF_CORE_LIB} PUBLIC
  ${PROJECT_SOURCE_DIR}/include 
)
get_property( ResultVariable TARGET ${EKF_CORE_LIB} PROPERTY INCLUDE_DIRECTORIES )
message( "KEF_CORE_LIB INCLUDE_DIRECTORIES ${ResultVariable}" )
target_link_libraries( ${EKF_CORE_LIB} 
  ${CMAKE_THREAD_LIBS_INIT}
  Eigen3::Eigen 
  ${EKF_UTILS_LIB}
)

# ===== Testing Executables =====
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/tester)

add_executable(ekf_offline_test tester/ekf_offline_test.cpp)
common_compile_prop( ekf_offline_test )
target_link_libraries(ekf_offline_test 
        ${CMAKE_THREAD_LIBS_INIT}
        Eigen3::Eigen
        ${EKF_CORE_LIB}
        ${EKF_UTILS_LIB}
        ${catkin_LIBRARIES}
)

add_executable(ekf_ros_test tester/ekf_ros_test.cpp)
common_compile_prop( ekf_ros_test )
target_link_libraries(ekf_ros_test
        ${CMAKE_THREAD_LIBS_INIT}
        Eigen3::Eigen
        ${EKF_CORE_LIB}
        ${EKF_UTILS_LIB}
        ${catkin_LIBRARIES}
        )