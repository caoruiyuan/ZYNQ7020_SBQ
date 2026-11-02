cmake_minimum_required(VERSION 3.16)
include(${CMAKE_SOURCE_DIR}/Empty_applicationExample.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/UserConfig.cmake)
set(APP_NAME DSP)
project(${APP_NAME})
find_package(common)
enable_language(C ASM CXX)

file(TO_CMAKE_PATH "$ENV{AX7020_SOURCE_ROOT}" SW_ROOT)
if(NOT EXISTS "${SW_ROOT}/common/ax7020_dual_packet.c")
    message(FATAL_ERROR "AX7020_SOURCE_ROOT is not set to 7020NEW/software")
endif()
set(_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${SW_ROOT}/common/ax7020_dual_packet.c
    ${SW_ROOT}/common/ax7020_waveform_trigger.c
    ${SW_ROOT}/analysis/ax7020_dual_analysis.c
    ${SW_ROOT}/analysis/q3_analysis.c
    ${SW_ROOT}/analysis/q3_calibration.c
)
collect(PROJECT_LIB_DEPS xilstandalone)
collect(PROJECT_LIB_DEPS xil)
collect(PROJECT_LIB_DEPS xiltimer)
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
target_link_libraries(${APP_NAME}.elf ${_linker_command} -Wl,--start-group ${_deps} -Wl,--end-group)
target_compile_definitions(${APP_NAME}.elf PUBLIC ${USER_COMPILE_DEFINITIONS})
target_compile_definitions(${APP_NAME}.elf PRIVATE USE_AMP=1)
target_include_directories(${APP_NAME}.elf PRIVATE
    ${SW_ROOT}/apps ${SW_ROOT}/analysis ${SW_ROOT}/common)
target_include_directories(${APP_NAME}.elf PUBLIC ${USER_INCLUDE_DIRECTORIES})
print_elf_size(CMAKE_SIZE ${APP_NAME})
