cmake_minimum_required(VERSION 3.13)
project(indi_wanderer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(INDI_MIN_VERSION 2.2.0)

find_package(INDI ${INDI_MIN_VERSION} QUIET)   # >= 2.2.0
if(NOT INDI_FOUND)
    # Fallback ohne CMake-Config: Version über pkg-config prüfen
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBINDI QUIET libindi>=${INDI_MIN_VERSION})
    endif()
    if(NOT LIBINDI_FOUND)
        message(FATAL_ERROR
            "libindi >= ${INDI_MIN_VERSION} nicht gefunden (installiert: siehe 'pkg-config --modversion libindi'). "
            "Native: libindi-dev aktualisieren oder INDI aus dem Quellcode/PPA installieren. "
            "Cross: PI_SYSROOT prüfen.")
    endif()
    find_path(INDI_INCLUDE_DIR indidevapi.h PATH_SUFFIXES libindi)
    find_library(INDI_LIBRARIES NAMES indidriver)
    message(STATUS "INDI version: ${LIBINDI_VERSION}")
endif()
if(NOT INDI_INCLUDE_DIR OR NOT INDI_LIBRARIES)
    message(FATAL_ERROR "INDI-Header oder -Bibliothek nicht gefunden. "
                        "Ggf. -DINDI_INCLUDE_DIR=... -DINDI_LIBRARIES=... setzen.")
endif()


message(STATUS "INDI include: ${INDI_INCLUDE_DIR}")
message(STATUS "INDI libs   : ${INDI_LIBRARIES}")

include_directories(${INDI_INCLUDE_DIR})

function(add_wanderer_driver exe variant)
    add_executable(${exe}
        src/wanderer_rotator_base.cpp
        src/wanderer_rotator_${variant}.cpp)
    target_include_directories(${exe} PRIVATE src)
    target_link_libraries(${exe} ${INDI_LIBRARIES})
    install(TARGETS ${exe} RUNTIME DESTINATION bin)
endfunction()

add_wanderer_driver(indi_wanderer_rotator_mini    mini)
add_wanderer_driver(indi_wanderer_lite_rotator    lite)
add_wanderer_driver(indi_wanderer_rotator_lite_v2 lite_v2)

install(FILES src/indi_wanderer_rotator.xml DESTINATION share/indi)
