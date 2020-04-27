# SPDX-License-Identifier: Apache-2.0

add_library(subsys__bluetooth INTERFACE)

target_include_directories(subsys__bluetooth INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(common)
add_subdirectory_ifdef(CONFIG_BT_HCI host)
add_subdirectory_ifdef(CONFIG_BT_SHELL shell)
add_subdirectory_ifdef(CONFIG_BT_CONN services)
add_subdirectory_ifdef(CONFIG_BT_MESH mesh)

if(CONFIG_BT_CTLR)
  if(CONFIG_BT_LL_SW_LEGACY OR CONFIG_BT_LL_SW_SPLIT)
    add_subdirectory(controller)
  endif()
endif()

if(CONFIG_ESP32_VHCI_CONTROLLER)
 # target_compile_options(subsys__bluetooth PRIVATE -Wno-implicit-fallthrough -Wno-unused-const-variable)
  add_subdirectory(esp_bluetooth/hci)
  target_link_libraries(subsys__bluetooth INTERFACE "-L${CMAKE_CURRENT_LIST_DIR}/esp_bluetooth/lib")
  # target_link_libraries(subsys__bluetooth INTERFACE btdm_app)
endif()
