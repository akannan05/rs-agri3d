#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rslidar_msg::rslidar_msg__rosidl_generator_c" for configuration "Release"
set_property(TARGET rslidar_msg::rslidar_msg__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rslidar_msg::rslidar_msg__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librslidar_msg__rosidl_generator_c.so"
  IMPORTED_SONAME_RELEASE "librslidar_msg__rosidl_generator_c.so"
  )

list(APPEND _cmake_import_check_targets rslidar_msg::rslidar_msg__rosidl_generator_c )
list(APPEND _cmake_import_check_files_for_rslidar_msg::rslidar_msg__rosidl_generator_c "${_IMPORT_PREFIX}/lib/librslidar_msg__rosidl_generator_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
