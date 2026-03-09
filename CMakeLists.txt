cmake_minimum_required(VERSION 3.16)

project(cerbo_gx_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(cerbo_gx_test
    src/cerbo_gx_test_main.cpp
)

# Try pkg-config/pkgconf first, since this is the most portable route for libmodbus
find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)
    pkg_check_modules(LIBMODBUS QUIET libmodbus)
endif()

if(LIBMODBUS_FOUND)
    message(STATUS "Found libmodbus via pkg-config")

    target_include_directories(cerbo_gx_test PRIVATE
        ${LIBMODBUS_INCLUDE_DIRS}
    )

    target_link_directories(cerbo_gx_test PRIVATE
        ${LIBMODBUS_LIBRARY_DIRS}
    )

    target_link_libraries(cerbo_gx_test PRIVATE
        ${LIBMODBUS_LIBRARIES}
    )

    target_compile_options(cerbo_gx_test PRIVATE
        ${LIBMODBUS_CFLAGS_OTHER}
    )

else()
    # Fallback for systems where pkg-config metadata is missing but headers/libs are installed
    find_path(LIBMODBUS_INCLUDE_DIR
        NAMES modbus/modbus.h
        PATH_SUFFIXES include
    )

    find_library(LIBMODBUS_LIBRARY
        NAMES modbus libmodbus
    )

    if(NOT LIBMODBUS_INCLUDE_DIR OR NOT LIBMODBUS_LIBRARY)
        message(FATAL_ERROR
            "Could not find libmodbus.\n"
            "On macOS, install both:\n"
            "  brew install libmodbus pkgconf\n"
            "On Debian/Ubuntu:\n"
            "  sudo apt install libmodbus-dev pkg-config\n"
        )
    endif()

    message(STATUS "Found libmodbus via manual search")

    target_include_directories(cerbo_gx_test PRIVATE
        ${LIBMODBUS_INCLUDE_DIR}
    )

    target_link_libraries(cerbo_gx_test PRIVATE
        ${LIBMODBUS_LIBRARY}
    )
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMOSQUITTO REQUIRED libmosquitto)

target_include_directories(cerbo_gx_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBMOSQUITTO_INCLUDE_DIRS}
)

target_link_directories(cerbo_gx_test PRIVATE
    ${LIBMOSQUITTO_LIBRARY_DIRS}
)

target_link_libraries(cerbo_gx_test PRIVATE
    ${LIBMOSQUITTO_LIBRARIES}
)

target_compile_options(cerbo_gx_test PRIVATE
    ${LIBMOSQUITTO_CFLAGS_OTHER}
)

target_include_directories(cerbo_gx_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)