#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "base_interfaces_demo::base_interfaces_demo__rosidl_generator_c" for configuration "Debug"
set_property(TARGET base_interfaces_demo::base_interfaces_demo__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(base_interfaces_demo::base_interfaces_demo__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libbase_interfaces_demo__rosidl_generator_c.so"
  IMPORTED_SONAME_DEBUG "libbase_interfaces_demo__rosidl_generator_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS base_interfaces_demo::base_interfaces_demo__rosidl_generator_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_base_interfaces_demo::base_interfaces_demo__rosidl_generator_c "${_IMPORT_PREFIX}/lib/libbase_interfaces_demo__rosidl_generator_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
