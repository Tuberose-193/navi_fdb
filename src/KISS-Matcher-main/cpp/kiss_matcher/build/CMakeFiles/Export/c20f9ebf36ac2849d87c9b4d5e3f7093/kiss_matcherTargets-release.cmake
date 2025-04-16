#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kiss_matcher::kiss_matcher_core" for configuration "Release"
set_property(TARGET kiss_matcher::kiss_matcher_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(kiss_matcher::kiss_matcher_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libkiss_matcher_core.so"
  IMPORTED_SONAME_RELEASE "libkiss_matcher_core.so"
  )

list(APPEND _cmake_import_check_targets kiss_matcher::kiss_matcher_core )
list(APPEND _cmake_import_check_files_for_kiss_matcher::kiss_matcher_core "${_IMPORT_PREFIX}/lib/libkiss_matcher_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
