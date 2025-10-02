cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(MecaBridge C CXX)

# Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(MecaBridge
  src/main.cpp
  src/frame_parser.cpp
  src/actuators.cpp
  src/enc.cpp
  src/esc_servo.cpp
  src/motors.cpp
  src/term.cpp
)

# enable usb output
pico_enable_stdio_usb(MecaBridge 1)

# Headers live in src/ (e.g., config.h)
target_include_directories(MecaBridge PRIVATE src)

# Link against required pico-sdk libs
target_link_libraries(MecaBridge
  pico_stdlib
  hardware_pwm
  hardware_gpio
  hardware_sync
  hardware_timer
)
# erlauben: reboot via 1200-baud und via Vendor-Interface (picotool)
target_compile_definitions(MecaBridge PRIVATE
  PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
)


# USB stdio on, UART off
pico_enable_stdio_usb(MecaBridge 1)
pico_enable_stdio_uart(MecaBridge 0)

# Create UF2/ELF/bin
pico_add_extra_outputs(MecaBridge)

# Allow `make build` in the generated build directory to build the firmware
add_custom_target(build DEPENDS MecaBridge)

# --- convenience targets ---
add_custom_target(flash
  COMMAND picotool load -f ${CMAKE_CURRENT_BINARY_DIR}/MecaBridge.uf2
  DEPENDS MecaBridge
  COMMENT "Flashing MecaBridge.uf2 via picotool"
)
