#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "csvutils::csvutils" for configuration ""
set_property(TARGET csvutils::csvutils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(csvutils::csvutils PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcsvutils.so"
  IMPORTED_SONAME_NOCONFIG "libcsvutils.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS csvutils::csvutils )
list(APPEND _IMPORT_CHECK_FILES_FOR_csvutils::csvutils "${_IMPORT_PREFIX}/lib/libcsvutils.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
