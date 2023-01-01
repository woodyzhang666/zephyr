# SPDX-License-Identifier: Apache-2.0

add_library(subsys__debugger INTERFACE)

target_include_directories(subsys__debugger INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(transport)
add_subdirectory_ifdef(CONFIG_DAPLINK daplink)

zephyr_include_directories(${ZEPHYR_BASE}/subsys/debugger)
