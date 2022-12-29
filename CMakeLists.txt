cmake_minimum_required(VERSION 2.8)
project(sfm-cpp)
set(CMAKE_CXX_STANDARD 14)
# opencv dependencies
# set(OpenCV_DIR /usr/share/OpenCV)
find_package(OpenCV REQUIRED)
include_directories(( ${OpenCV_INCLUDE_DIRS}))

#adding googletest

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
# LIST(APPEND CMAKE_MODULE_PATH /home/kartik/g2o/cmake_modules)
set(G2O_DIR /Users/kartikmadhira/g2o/cmake_modules)
find_package(G2O REQUIRED)


find_package(Boost REQUIRED COMPONENTS thread)
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

find_package(jsoncpp REQUIRED)
find_package(Boost 1.3 COMPONENTS filesystem) 
find_package(Eigen3 3.3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${G2O_INCLUDE_DIR})

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

include_directories(${Boost_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS}) 
find_package (glog 0.6.0 REQUIRED)


IF (G2O_INCLUDE_DIR)
    MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
  ENDIF (G2O_INCLUDE_DIR)

  FIND_LIBRARY(G2O_CORE_LIB             
    NAMES g2o_core 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_STUFF_LIB            
    NAMES g2o_stuff 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_TYPES_SLAM3D_LIB     
    NAMES g2o_types_slam3d 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_CHOLMOD_LIB   
    NAMES g2o_solver_cholmod 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_PCG_LIB       
    NAMES g2o_solver_pcg 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_SOLVER_CSPARSE_LIB   
    NAMES g2o_solver_csparse 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_INCREMENTAL_LIB      
    NAMES g2o_incremental 
    PATHS /usr/local /usr 
    PATH_SUFFIXES lib)
  FIND_LIBRARY(G2O_CSPARSE_EXTENSION_LIB
    NAMES g2o_csparse_extension
    PATHS /usr/local /usr
    PATH_SUFFIXES lib)

  SET(G2O_LIBRARIES ${G2O_CSPARSE_EXTENSION_LIB}
                    ${G2O_CORE_LIB}           
                    ${G2O_STUFF_LIB}          
                    ${G2O_TYPES_SLAM3D_LIB}   
                    ${G2O_SOLVER_CHOLMOD_LIB} 
                    ${G2O_SOLVER_PCG_LIB}     
                    ${G2O_SOLVER_CSPARSE_LIB} 
                    ${G2O_INCREMENTAL_LIB}                        
                    )
 
  IF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    SET(G2O_FOUND "YES")
    IF(NOT G2O_FIND_QUIETLY)
      MESSAGE(STATUS "Found libg2o: ${G2O_LIBRARIES}")
    ENDIF(NOT G2O_FIND_QUIETLY)
  ELSE(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
    IF(NOT G2O_LIBRARIES)
      IF(G2O_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find libg2o!")
      ENDIF(G2O_FIND_REQUIRED)
    ENDIF(NOT G2O_LIBRARIES)

    IF(NOT G2O_INCLUDE_DIR)
      IF(G2O_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find g2o include directory!")
      ENDIF(G2O_FIND_REQUIRED)
    ENDIF(NOT G2O_INCLUDE_DIR)
  ENDIF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)





add_executable(sfm-cpp src/temp.cc src/Utils.cc src/Image.cc src/BriefDescriptor.cc  src/FastDetector.cc src/3DHandler.cc)
target_link_libraries(sfm-cpp ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES})
# get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

add_executable(
  g2otest
  src/test.cc
)


target_link_libraries(g2otest ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIB} ${G2O_STUFF_LIBRARY} ${G2O_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} glog::glog)

# Link CHOLMOD if available, CSparse otherwise


enable_testing()

add_executable(
  UtilsTest
  tests/UtilsTest.cc src/Utils.cc
)
target_link_libraries(
  UtilsTest
  GTest::gtest_main ${Boost_LIBRARIES} ${OpenCV_LIBS}
)

add_executable(
  OptimizerTest
  tests/OptimizerTest.cc src/Optimizer.cc src/3DHandler.cc src/Utils.cc src/FastDetector.cc src/Image.cc src/LoopHandler.cc src/BriefDescriptor.cc
)
target_link_libraries(
  OptimizerTest
  GTest::gtest_main ${Boost_LIBRARIES} ${OpenCV_LIBS}
  jsoncpp_lib 
  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIB} ${G2O_STUFF_LIBRARY} ${G2O_LIBRARIES} 
  glog::glog
)




add_executable(
  ImageTest
  tests/ImageTest.cc src/Utils.cc src/FastDetector.cc src/Image.cc src/LoopHandler.cc src/BriefDescriptor.cc
)
target_link_libraries(
  ImageTest
  GTest::gtest_main ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES}
)

add_executable(
  BriefDescriptorTest
  tests/BriefDescriptorTest.cc src/Utils.cc src/FastDetector.cc src/Image.cc src/LoopHandler.cc src/BriefDescriptor.cc
)
target_link_libraries(
  BriefDescriptorTest
  GTest::gtest_main ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES}
)




add_executable(
  LoopHandlerTest
  tests/LoopHandlerTest.cc src/Utils.cc src/LoopHandler.cc src/FastDetector.cc src/Image.cc  src/BriefDescriptor.cc src/3DHandler.cc src/Image.cc
)
target_link_libraries(
  LoopHandlerTest
  GTest::gtest_main ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES}
)

add_executable(
  FastDetectorTest
  tests/FastDetectorTest.cc src/Utils.cc src/FastDetector.cc src/LoopHandler.cc src/Image.cc src/BriefDescriptor.cc
)
target_link_libraries(
  FastDetectorTest
  GTest::gtest_main ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES}
)

add_executable(
  3DHandlerTest
  tests/3DHandlerTest.cc src/Utils.cc src/FastDetector.cc src/Image.cc src/LoopHandler.cc src/BriefDescriptor.cc src/3DHandler.cc
)
target_link_libraries(
  3DHandlerTest
  GTest::gtest_main ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES}
)


include(GoogleTest)
