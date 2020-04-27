# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(esp_zephyr_hci.c)
zephyr_library_sources(bt.c)
zephyr_library_link_libraries(subsys__bluetooth)
