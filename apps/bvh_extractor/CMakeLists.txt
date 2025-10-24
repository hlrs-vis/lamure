############################################################
# CMake Build Script for the bvh_extractor executable

link_directories(${SCHISM_LIBRARY_DIRS})

include_directories(${REND_INCLUDE_DIR} 
                    ${COMMON_INCLUDE_DIR})

include_directories(SYSTEM ${SCHISM_INCLUDE_DIRS}
						   ${Boost_INCLUDE_DIR})


InitApp(${CMAKE_PROJECT_NAME}_bvh_extractor)

############################################################
# Libraries

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_LIBS}
    ${REND_LIBRARY}
    optimized ${Boost_FILESYSTEM_LIBRARY_RELEASE}
    optimized ${SCHISM_CORE_LIBRARY} #debug ${SCHISM_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_UTIL_LIBRARY}
    optimized ${Boost_FILESYSTEM_LIBRARY_RELEASE}
    ${OpenGL_LIBRARIES} 
    ${OPENGL_glx_LIBRARY} 
    ${GLUT_LIBRARY}
${OSG_LIBRARIES}
    ${E57_LIBRARY}
    ${XERCESC_LIBRARIES}
    )

