cmake_minimum_required(VERSION 3.16)
include(${CMAKE_SOURCE_DIR}/Empty_applicationExample.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/UserConfig.cmake)
set(APP_NAME LVGL)
project(${APP_NAME})
find_package(common)
enable_language(C ASM CXX)

file(TO_CMAKE_PATH "$ENV{AX7020_SOURCE_ROOT}" SW_ROOT)
set(LVGL_ROOT ${SW_ROOT}/software.vitis/LVGL)
set(TINYUSB_ROOT ${SW_ROOT}/third_party/tinyusb/src)
if(NOT EXISTS "${LVGL_ROOT}/lvgl/src/lvgl.h")
    message(FATAL_ERROR "AX7020_SOURCE_ROOT is not set to 7020NEW/software")
endif()
if(NOT EXISTS "${TINYUSB_ROOT}/portable/ehci/ehci.c")
    message(FATAL_ERROR "TinyUSB v0.17.0 is missing")
endif()
file(GLOB_RECURSE LVGL_SOURCES CONFIGURE_DEPENDS
    "${LVGL_ROOT}/lvgl/src/*.c")
add_library(lvgl STATIC ${LVGL_SOURCES})
target_include_directories(lvgl SYSTEM PUBLIC
    ${LVGL_ROOT}/lvgl ${LVGL_ROOT}/src)
target_compile_definitions(lvgl PUBLIC LV_CONF_INCLUDE_SIMPLE LV_KCONFIG_IGNORE)
target_compile_options(lvgl PRIVATE -O2 -Wno-unused-parameter)

set(_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ax7020_ui.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ax7020_calibration_store.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ax7020_usb_keyboard.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ax7020_usb_keys.c
    ${TINYUSB_ROOT}/tusb.c
    ${TINYUSB_ROOT}/common/tusb_fifo.c
    ${TINYUSB_ROOT}/host/usbh.c
    ${TINYUSB_ROOT}/host/hub.c
    ${TINYUSB_ROOT}/class/hid/hid_host.c
    ${TINYUSB_ROOT}/portable/ehci/ehci.c
    ${LVGL_ROOT}/src/fonts/ui_font_cn_16.c
    ${LVGL_ROOT}/src/fonts/ui_font_cn_20.c
    ${LVGL_ROOT}/src/fonts/ui_font_scope_extra.c
    ${LVGL_ROOT}/src/lv_port_zynq.c
    ${LVGL_ROOT}/src/lv_port_touch.c
    ${LVGL_ROOT}/src/gt911.c
)
collect(PROJECT_LIB_DEPS xilstandalone)
collect(PROJECT_LIB_DEPS xil)
collect(PROJECT_LIB_DEPS xiltimer)
collect(PROJECT_LIB_DEPS xilffs)
collect(PROJECT_LIB_DEPS gcc)
collect(PROJECT_LIB_DEPS c)
collect(PROJECT_LIB_DEPS m)
collector_list(_deps PROJECT_LIB_DEPS)
link_directories(${CMAKE_LIBRARY_PATH})
linker_gen("${CMAKE_SOURCE_DIR}/linker_files/")
string(APPEND CMAKE_C_FLAGS ${USER_COMPILE_OPTIONS})
string(APPEND CMAKE_C_LINK_FLAGS ${USER_LINK_OPTIONS})
add_dependency_on_bsp(_sources)
add_executable(${APP_NAME}.elf ${_sources})
set_target_properties(${APP_NAME}.elf PROPERTIES LINK_DEPENDS ${USER_LINKER_SCRIPT})
set(_linker_command "-Wl,-T -Wl,\"${USER_LINKER_SCRIPT}\" -L\"${CMAKE_SOURCE_DIR}/\" -L\"${CMAKE_LIBRARY_PATH}/\" -L\"${USER_LINK_DIRECTORIES}/\"")
target_link_libraries(${APP_NAME}.elf lvgl ${_linker_command} -Wl,--start-group ${_deps} -Wl,--end-group)
target_compile_definitions(${APP_NAME}.elf PUBLIC ${USER_COMPILE_DEFINITIONS})
if("$ENV{AX7020_DISPLAY_DIAGNOSTICS}" STREQUAL "1")
    target_compile_definitions(${APP_NAME}.elf PRIVATE AX7020_DISPLAY_DIAGNOSTICS=1)
endif()
target_include_directories(${APP_NAME}.elf PRIVATE
    ${SW_ROOT}/apps ${LVGL_ROOT}/src ${LVGL_ROOT}/lvgl
    ${TINYUSB_ROOT} ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${APP_NAME}.elf PUBLIC ${USER_INCLUDE_DIRECTORIES})
print_elf_size(CMAKE_SIZE ${APP_NAME})
