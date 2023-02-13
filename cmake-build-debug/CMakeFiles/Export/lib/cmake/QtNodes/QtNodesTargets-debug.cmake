#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "QtNodes::QtNodes" for configuration "Debug"
set_property(TARGET QtNodes::QtNodes APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(QtNodes::QtNodes PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libQtNodes.so"
  IMPORTED_SONAME_DEBUG "libQtNodes.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS QtNodes::QtNodes )
list(APPEND _IMPORT_CHECK_FILES_FOR_QtNodes::QtNodes "${_IMPORT_PREFIX}/lib/libQtNodes.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
