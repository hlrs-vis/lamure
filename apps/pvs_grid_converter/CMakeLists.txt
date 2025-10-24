############################################################
# CMake Build Script for the pvs_grid_converter executable

# Falls du (noch) keine imported targets für SCHISM hast, einmal setzen:
link_directories(${SCHISM_LIBRARY_DIRS})

include_directories(${PVS_COMMON_INCLUDE_DIR}
                    ${PVS_PREPROCESSING_INCLUDE_DIR}
                    ${REND_INCLUDE_DIR}
                    ${COMMON_INCLUDE_DIR}
                    ${FREEGLUT_INCLUDE_DIR}
                    ${FREEIMAGE_INCLUDE_DIR}
			        ${LAMURE_CONFIG_DIR})

include_directories(SYSTEM ${SCHISM_INCLUDE_DIRS}
						   ${Boost_INCLUDE_DIR})

link_directories(${SCHISM_LIBRARY_DIRS})

InitApp(${CMAKE_PROJECT_NAME}_pvs_grid_converter)

############################################################
# Libraries

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_LIBS}
    ${PVS_COMMON_LIBRARY}
    ${PVS_PREPROCESSING_LIBRARY}
    ${REND_LIBRARY}
    ${OpenGL_LIBRARIES} 
    ${FREEGLUT_LIBRARY}
    ${FREEIMAGE_LIBRARY}
    -lfreetype
    optimized ${SCHISM_CORE_LIBRARY} debug ${SCHISM_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_CORE_LIBRARY} debug ${SCHISM_GL_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_UTIL_LIBRARY} debug ${SCHISM_GL_UTIL_LIBRARY_DEBUG}
${OSG_LIBRARIES}
    ${E57_LIBRARY}
    ${XERCESC_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME} lamure_pvs_preprocessing lamure_common)

MsvcPostBuild(${PROJECT_NAME})
