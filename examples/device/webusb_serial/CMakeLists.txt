# Override CMake for TinyUSB webusb_serial example
cmake_minimum_required(VERSION 3.20)

# Use TinyUSB family support helpers
include(${CMAKE_CURRENT_SOURCE_DIR}/../../../hw/bsp/family_support.cmake)

set(PROJECT "drone-config-usb")


project(${PROJECT} C CXX ASM)

# Initialize project for detected FAMILY/BOARD
family_initialize_project(${PROJECT} ${CMAKE_CURRENT_LIST_DIR})

add_executable(${PROJECT})

target_sources(${PROJECT} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/main.c
  ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
)

target_include_directories(${PROJECT} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_definitions(${PROJECT} PUBLIC
  WEBUSB_URL="brianharper.us/drone/index.html"
)

# Configure for device example without RTOS
family_configure_device_example(${PROJECT} noos)
