cmake_minimum_required(VERSION 3.16)

project(cerbo_gx_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(cerbo_gx_test
    src/cerbo_gx_test_main.cpp
)

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