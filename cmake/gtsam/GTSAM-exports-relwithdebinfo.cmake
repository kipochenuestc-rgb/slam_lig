#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CppUnitLite" for configuration "RelWithDebInfo"
set_property(TARGET CppUnitLite APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(CppUnitLite PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libCppUnitLite.a"
  )


# Import target "gtsam" for configuration "RelWithDebInfo"
set_property(TARGET gtsam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(gtsam PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libgtsam.so.4.2.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libgtsam.so.4"
  )

list(APPEND _cmake_import_check_targets gtsam )
list(APPEND _cmake_import_check_files_for_gtsam "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libgtsam.so.4.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
