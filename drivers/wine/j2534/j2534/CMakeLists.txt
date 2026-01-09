cmake_minimum_required (VERSION 3.8)
set (CMAKE_C_STANDARD 99)

find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATHS /usr /usr/local /opt "${CMAKE_SOURCE_DIR}/mslib" PATH_SUFFIXES libusb-1.0)
SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".dll" ".so")
find_library(LIBUSB_LIBRARY NAMES usb-1.0 PATHS /usr /usr/local /opt "${CMAKE_SOURCE_DIR}/mslib")

include_directories(${LIBUSB_INCLUDE_DIR})

if(MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    set(BUILD_SHARED_LIBS TRUE)
    set(INSTDIR "C:/Program Files (x86)/WQCAN")
else()
    set(INSTDIR "~/.passthru/")
    install(FILES "${CMAKE_SOURCE_DIR}/misc/wqcan.json" DESTINATION ${INSTDIR})
endif()

add_library(j2534drv SHARED "j2534drv.c" "j2534translate.c" "log.c" "utils.c")
target_link_libraries(j2534drv ${LIBUSB_LIBRARY})
target_compile_options(j2534drv PUBLIC -DLOG_USE_COLOR )
install(TARGETS j2534drv DESTINATION  ${INSTDIR})
