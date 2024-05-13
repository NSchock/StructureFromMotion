cmake_minimum_required(VERSION 3.28.1)
set (CMAKE_CXX_STANDARD 23)

project(StructureFromMotion VERSION 0.1.0 LANGUAGES C CXX)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Eigen3 3.3 REQUIRED NO_MODULE )
include_directories( ${Eigen3_INCLUDE_DIRS} )

find_package( GTEST CONFIG REQUIRED )
include_directories( ${GTEST_INCLUDE_DIRS} )


set(HEADERS
    include/image_utils.h
    include/geometric_verification.h
    include/feature_matching.h
    include/normalization.h
)
set(SOURCES
    src/image_utils.cpp
    src/geometric_verification.cpp
    src/feature_matching.cpp
    src/normalization.cpp
)
set(TESTS
    test/feature_matching_test.cpp
    test/normalization_test.cpp
)

enable_testing()

# Add main.cpp to compile program instead of test
# TODO: better testing integration
add_executable(StructureFromMotion ${SOURCES} ${HEADERS} ${TESTS})
target_link_libraries( StructureFromMotion
                       ${OpenCV_LIBS}
                       Eigen3::Eigen
                       GTest::gtest_main
)
target_include_directories( StructureFromMotion PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" )

include(CTest)
include(GoogleTest)
gtest_discover_tests(StructureFromMotion)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

