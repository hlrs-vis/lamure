############################################################
# CMake Build Script for the common library

cmake_minimum_required(VERSION 2.8.7)
project(${CMAKE_PROJECT_NAME}_common)

set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/common/include)
set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/common/src)

configure_file(
  include/lamure/config.h.in
  ${LAMURE_CONFIG_DIR}/lamure/config.h
  )

file(GLOB_RECURSE PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE PROJECT_INCLUDES ${PROJECT_INCLUDE_DIR}/*.h)

file(GLOB PLATFORM_INCLUDE "${PROJECT_INCLUDE_DIR}/lamure/platform.h")

SOURCE_GROUP("common" FILES ${PROJECT_SOURCES})
SOURCE_GROUP("common" FILES ${PROJECT_INCLUDES})
SOURCE_GROUP("common" FILES ${PLATFORM_INCLUDE})

include_directories(${PROJECT_INCLUDE_DIR}
                    ${PROJECT_BINARY_DIR}
                    ${LAMURE_CONFIG_DIR})

include_directories(SYSTEM ${SCHISM_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIR})

link_directories(${SCHISM_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED ${PROJECT_INCLUDES} ${PROJECT_SOURCES} ${LAMURE_CONFIG_DIR}/lamure/config.h)

IF (MSVC)
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D LAMURE_COMMON_LIBRARY")
ENDIF(MSVC)

set(COMMON_INCLUDE_DIR ${PROJECT_INCLUDE_DIR} ${PROJECT_BINARY_DIR} ${PB_INCLUDE_DIR} ${PROJECT_BINARY_DIR} PARENT_SCOPE)
set(COMMON_LIBRARY ${PROJECT_NAME} PARENT_SCOPE)
set(COMMON_LIBRARY ${PROJECT_NAME})

############################################################
# Libraries

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_LIBS}
    optimized ${Boost_TIMER_LIBRARY_RELEASE} debug ${Boost_TIMER_LIBRARY_DEBUG}
    optimized ${Boost_CHRONO_LIBRARY_RELEASE} debug ${Boost_CHRONO_LIBRARY_DEBUG}
    optimized ${Boost_SYSTEM_LIBRARY_RELEASE} debug ${Boost_SYSTEM_LIBRARY_DEBUG}
    optimized ${Boost_FILESYSTEM_LIBRARY_RELEASE} debug ${Boost_FILESYSTEM_LIBRARY_DEBUG}
    optimized ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE} debug ${Boost_PROGRAM_OPTIONS_LIBRARY_DEBUG}
    )

set_source_files_properties(${PB_SOURCES} PROPERTIES GENERATED TRUE)

###############################################################################
# install 
###############################################################################

IF (MSVC)
  install (TARGETS ${PROJECT_NAME}
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS ${PROJECT_NAME}
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
ENDIF (MSVC)

# install header files and license
install(DIRECTORY ${PROJECT_INCLUDE_DIR}/lamure DESTINATION include FILES_MATCHING PATTERN "*.h")
install(FILES ${LAMURE_CONFIG_DIR}/lamure/config.h DESTINATION include/lamure)
install(FILES ${CMAKE_SOURCE_DIR}/license.txt DESTINATION .)
