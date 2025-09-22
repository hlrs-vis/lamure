##############################################################################
# search paths
##############################################################################
SET(E57_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/e57/include/e57
)

SET(E57_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/e57/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for E57")

SET(E57_LIBRARY_FILENAMES "")

FOREACH (_SEARCH_DIR ${E57_LIBRARY_SEARCH_DIRS})
    FILE(GLOB _RESULT RELATIVE ${_SEARCH_DIR} "${_SEARCH_DIR}/E57RefImpl*" )
    IF (_RESULT) 
        FOREACH(_FILEPATH ${_RESULT})
            get_filename_component(_LIBNAME ${_FILEPATH} NAME)
            SET(E57_LIBRARY_FILENAMES ${E57_LIBRARY_FILENAMES} ${_LIBNAME})
        ENDFOREACH()
    ENDIF ()
ENDFOREACH()

find_path(E57_INCLUDE_DIR NAMES E57Foundation.h E57Simple.h constants.h basictypes.h gnss_error.h time_conversion.h PATHS ${E57_INCLUDE_SEARCH_DIRS})

find_library(E57_LIBRARY NAMES E57RefImpl.lib E57RefImpl-d.lib ${E57_LIBRARY_FILENAMES} PATHS ${E57_LIBRARY_SEARCH_DIRS})

##############################################################################
# verify
##############################################################################
IF ( E57_INCLUDE_DIR AND E57_LIBRARY)
  MESSAGE(STATUS "--  found matching e57 version")
ELSE()
  MESSAGE(FATAL_ERROR "find_e57.cmake: unable to find e57 library.")
ENDIF ()