add_library(pico_uvc_camera INTERFACE)

target_sources(pico_uvc_camera INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/uvc_camera.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/uvc_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/uvc_descriptors.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/uvc_stream.cpp
)

target_include_directories(pico_uvc_camera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pico_uvc_camera INTERFACE pico_logger pico_simple_util)

if (PICO_OS_PLATFORM STREQUAL "pico")
  target_sources(pico_uvc_camera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/pico/uvc_transport_rp2350.cpp)
  target_link_libraries(pico_uvc_camera INTERFACE hardware_irq hardware_resets hardware_sync)
else()
  target_sources(pico_uvc_camera INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/linux/uvc_transport_libusb.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/linux/uvc_transport_replay.cpp
  )
  target_include_directories(pico_uvc_camera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/linux)

  find_package(PkgConfig QUIET)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(LIBUSB QUIET IMPORTED_TARGET libusb-1.0)
  endif()
  if (LIBUSB_FOUND)
    target_link_libraries(pico_uvc_camera INTERFACE PkgConfig::LIBUSB)
  else()
    message(WARNING "pico_uvc_camera: libusb-1.0 not found (libusb-1.0-0-dev in os/.devcontainer/linux.Dockerfile)")
  endif()
endif()

set(PICO_UVC_CAMERA_COMMANDS ${CMAKE_CURRENT_SOURCE_DIR}/commands CACHE INTERNAL "pico_uvc_camera commands")
