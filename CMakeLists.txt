cmake_minimum_required(VERSION 3.28.1)
set (CMAKE_CXX_STANDARD 23)

project(StructureFromMotion VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(HEADERS
    include/image_utils.h
)
set(SOURCES
    src/image_utils.cpp
    main.cpp
)

add_executable(StructureFromMotion ${SOURCES} ${HEADERS})
target_link_libraries( StructureFromMotion ${OpenCV_LIBS} )
target_include_directories( StructureFromMotion PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
