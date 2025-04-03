cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS components/freemodbus_params)
set(EXTRA_COMPONENT_DIRS components/hexnet)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rgb_panel)
