##############################################################################
# search paths
##############################################################################
SET(OSG_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/OpenSceneGraph/lib
  ${GLOBAL_EXT_DIR}/OpenSceneGraph/bin
)

SET(OSG_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/OpenSceneGraph/include
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for OSG")

SET(OSG_LIBRARY_FILENAMES "")

FOREACH (_SEARCH_DIR ${OSG_LIBRARY_SEARCH_DIRS})
    FILE(GLOB _RESULT RELATIVE ${_SEARCH_DIR} "${_SEARCH_DIR}/osg*" )
    IF (_RESULT) 
        FOREACH(_FILEPATH ${_RESULT})
            get_filename_component(_LIBNAME ${_FILEPATH} NAME)
            SET(OSG_LIBRARY_FILENAMES ${OSG_LIBRARY_FILENAMES} ${_LIBNAME})
        ENDFOREACH()
    ENDIF ()
ENDFOREACH()

find_path(OSG_INCLUDE_DIR NAMES osg.h PATHS ${OSG_INCLUDE_SEARCH_DIRS})

find_library(OSG_LIBRARY NAMES ${OSG_LIBRARY_FILENAMES} PATHS ${OSG_LIBRARY_SEARCH_DIRS})

##############################################################################
# verify
##############################################################################
IF ( OSG_INCLUDE_DIR AND OSG_LIBRARY)
  MESSAGE(STATUS "--  found matching osg version")
ELSE()
  MESSAGE(FATAL_ERROR "find_osg.cmake: unable to find OSG library.")
ENDIF ()