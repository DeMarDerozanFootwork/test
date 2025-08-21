
if(${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  add_subdirectory(cmake/am62a)
else()
  add_subdirectory(cmake/tda4x)
endif()
