############################################################
# CMake Build Script for the vt_stitch executable

include_directories(${REND_INCLUDE_DIR}
        ${COMMON_INCLUDE_DIR}
        ${FREEIMAGE_INCLUDE_DIR})

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

InitApp(${CMAKE_PROJECT_NAME}_vt_stitch)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

file(GLOB PROJECT_INCLUDES ${PROJECT_SOURCE_DIR}/*.h)
file(GLOB PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/*.cpp)

############################################################
# Libraries
if(WIN32)
else(WIN32)
set(CPPFILESYSTEM stdc++fs)# C++17
endif(WIN32)

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_LIBS}
        ${REND_LIBRARY}
        optimized ${Boost_REGEX_LIBRARY_RELEASE} debug ${Boost_REGEX_LIBRARY_DEBUG}
        ${CPPFILESYSTEM} 
        )

add_dependencies(${PROJECT_NAME} lamure_rendering lamure_common lamure_pvs_common)

MsvcPostBuild(${PROJECT_NAME})

