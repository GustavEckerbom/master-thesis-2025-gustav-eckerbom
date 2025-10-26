cmake_minimum_required(VERSION 3.15)
project(usb_event_viewer C)

set(CMAKE_C_STANDARD 11)

# SDL3 paths (corrected)
set(SDL3_INCLUDE_DIR "C:/libs/SDL3-3.2.10/include")
set(SDL3_LIB_DIR "C:/libs/SDL3-3.2.10/lib/x64")

add_executable(usb_event_viewer src/main.c)

# Include and link SDL
target_include_directories(usb_event_viewer PRIVATE ${SDL3_INCLUDE_DIR})
target_link_directories(usb_event_viewer PRIVATE ${SDL3_LIB_DIR})
target_link_libraries(usb_event_viewer PRIVATE SDL3)

# Copy SDL3.dll after build
add_custom_command(TARGET usb_event_viewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${SDL3_LIB_DIR}/SDL3.dll"
    "$<TARGET_FILE_DIR:usb_event_viewer>"
)
