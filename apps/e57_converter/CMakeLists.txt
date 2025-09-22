############################################################
# CMake Build Script for the e57_converter executable


link_directories(${SCHISM_LIBRARY_DIRS})

include_directories(${REND_INCLUDE_DIR}
                    ${PREPROC_INCLUDE_DIR}
                    ${COMMON_INCLUDE_DIR}
                    ${CGAL_INCLUDE_DIR}
                    ${GMP_INCLUDE_DIR}
                    ${MPFR_INCLUDE_DIR}
                    ${E57RefImpl_INCLUDE_DIR}
                    ${OSG_INCLUDE_DIR}
                    ${XERCES_INCLUDE_DIR})

include_directories(SYSTEM ${SCHISM_INCLUDE_DIRS}
						   ${Boost_INCLUDE_DIR})


InitApp(${CMAKE_PROJECT_NAME}_e57_converter)

############################################################
# Libraries

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_LIBS}
        ${COMMON_LIBRARY}
        ${PREPROC_LIBRARY}
        optimized ${Boost_THREAD_LIBRARY_RELEASE} debug ${Boost_THREAD_LIBRARY_DEBUG}
        optimized ${E57RefImpl_LIBRARY_RELEASE} debug ${E57RefImpl_LIBRARY_DEBUG}
        optimized ${OSG_LIBRARY_RELEASE} debug ${OSG_LIBRARY_DEBUG}
        optimized ${XERCES_LIBRARY_RELEASE} debug ${XERCES_LIBRARY_DEBUG}
        )