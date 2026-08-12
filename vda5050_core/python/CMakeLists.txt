find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(_core
  src/core.cpp
  src/rmf_migration.cpp
  src/transport.cpp
  src/master.cpp
)

target_link_libraries(_core
  PUBLIC
    vda5050_core::layout
    vda5050_core::validation
    vda5050_core::logger
    vda5050_core::client
    vda5050_core::master
    vda5050_core::transport
  PRIVATE
    Python3::Python
)

target_include_directories(_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(SKBUILD)
  # Install for scikit-build-core
  install(TARGETS
    _core
    DESTINATION .
  )

  # Manual RPATH patching
  if(APPLE)
    set(origin_token "@loader_path")
  else()
    set(origin_token "$ORIGIN")
  endif()
  set_property(TARGET _core
    PROPERTY
      INSTALL_RPATH "${origin_token}/lib"
  )
  set_target_properties(
      vda5050_logger
      vda5050_validation
      vda5050_transport
      vda5050_execution
      vda5050_client
      vda5050_layout
      vda5050_master
    PROPERTIES
      INSTALL_RPATH "${origin_token}"
  )
else()
install(TARGETS
  _core
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
endif()
