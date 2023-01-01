# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_DEBUGGER_TRANSPORT_USB usb_vsc.c)
zephyr_library_link_libraries(subsys__debugger)

