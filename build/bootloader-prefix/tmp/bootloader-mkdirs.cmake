# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/nakau/esp/v5.3/esp-idf/components/bootloader/subproject"
  "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader"
  "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix"
  "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix/tmp"
  "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix/src/bootloader-stamp"
  "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix/src"
  "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "G:/Shared drives/ARGE/02 Caravan/03 Software Design/01-VangoPanel/V0.04/Vango_ESPIDF/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
