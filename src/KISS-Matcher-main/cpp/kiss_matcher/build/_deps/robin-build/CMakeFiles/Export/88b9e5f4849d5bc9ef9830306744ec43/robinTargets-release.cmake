#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robin::robin" for configuration "Release"
set_property(TARGET robin::robin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robin::robin PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobin.a"
  )

list(APPEND _cmake_import_check_targets robin::robin )
list(APPEND _cmake_import_check_files_for_robin::robin "${_IMPORT_PREFIX}/lib/librobin.a" )

# Import target "robin::pmc" for configuration "Release"
set_property(TARGET robin::pmc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robin::pmc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpmc.a"
  )

list(APPEND _cmake_import_check_targets robin::pmc )
list(APPEND _cmake_import_check_files_for_robin::pmc "${_IMPORT_PREFIX}/lib/libpmc.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
