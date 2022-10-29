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
find_package(jsoncpp REQUIRED)
find_package(Boost 1.3 COMPONENTS filesystem) 
include_directories(${Boost_INCLUDE_DIRS}) 

add_executable(sfm-cpp src/temp.cc src/Image.cc src/LoopHandler.cc include/Utils.hpp)
target_link_libraries(sfm-cpp ${OpenCV_LIBS} jsoncpp_lib ${Boost_LIBRARIES})
# get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

enable_testing()

add_executable(
  UtilsTest
  tests/UtilsTest.cc
)
target_link_libraries(
  UtilsTest
  GTest::gtest_main ${OpenCV_LIBS} ${Boost_LIBRARIES}
)

add_executable(
  LoopHandlerTest
  tests/LoopHandlerTest.cc
)
target_link_libraries(
  LoopHandlerTest
  GTest::gtest_main ${OpenCV_LIBS} ${Boost_LIBRARIES}
)

include(GoogleTest)
gtest_discover_tests(UtilsTest LoopHandlerTest)