cmake_minimum_required(VERSION 3.28.1)
set (CMAKE_CXX_STANDARD 23)

project(StructureFromMotion VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Eigen3 3.3 REQUIRED NO_MODULE )
include_directories( ${Eigen3_INCLUDE_DIRS} )

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
    main.cpp
)

add_executable(StructureFromMotion ${SOURCES} ${HEADERS})
target_link_libraries( StructureFromMotion ${OpenCV_LIBS} )
target_link_libraries( StructureFromMotion Eigen3::Eigen )
target_include_directories( StructureFromMotion PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
