cmake_minimum_required(VERSION 2.8)
project(sfm-cpp)
set(CMAKE_CXX_STANDARD 14)
# opencv dependencies
set(OpenCV_DIR /usr/share/OpenCV)
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
LIST(APPEND CMAKE_MODULE_PATH /home/kartik/g2o/cmake_modules)
set(G2O_DIR /home/kartik/g2o/cmake_modules)
find_package(G2O REQUIRED)

#find_package(Boost REQUIRED COMPONENTS thread)
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

find_package(jsoncpp REQUIRED)
find_package(Boost 1.3 COMPONENTS filesystem) 
find_package(Eigen3 3.3 REQUIRED)
find_package(Sophus REQUIRED)


add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

include_directories(${Boost_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS}) 
find_package (glog 0.6.0 REQUIRED)


add_executable(sfm-cpp src/temp.cc src/Utils.cc src/Image.cc src/BriefDescriptor.cc src/LoopHandler.cc src/FastDetector.cc src/3DHandler.cc)
target_link_libraries(sfm-cpp ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES} Sophus::Sophus)
# get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

add_executable(
  g2otest
  src/test.cc
)


target_link_libraries(g2otest g2o_core g2o_stuff cholmod  g2o_types_sba
g2o_types_sim3
g2o_types_slam2d
g2o_types_slam3d g2o_types_icp ${CHOLMOD_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} glog::glog)

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
  Sophus::Sophus jsoncpp_lib 
  g2o_core g2o_stuff cholmod  g2o_types_sba
  g2o_types_sim3
  g2o_types_slam2d
  g2o_types_slam3d g2o_types_icp
  glog::glog
)




add_executable(
  ImageTest
  tests/ImageTest.cc src/Utils.cc src/FastDetector.cc src/Image.cc src/LoopHandler.cc
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
  tests/LoopHandlerTest.cc src/Utils.cc src/LoopHandler.cc src/FastDetector.cc src/Image.cc
)
target_link_libraries(
  LoopHandlerTest
  GTest::gtest_main ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES}
)

add_executable(
  FastDetectorTest
  tests/FastDetectorTest.cc src/Utils.cc src/FastDetector.cc src/LoopHandler.cc src/Image.cc
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
gtest_discover_tests(UtilsTest LoopHandlerTest FastDetectorTest ImageTest BriefDescriptorTest 3DHandlerTest)