# /usr/lib/x86_64-linux-gnu/pkgconfig/libmodbus.pc
cmake_minimum_required(VERSION 3.6)

project(dfrobot_sen0358)

find_package(PkgConfig REQUIRED)

set(ENV{PKG_CONFIG_PATH} "/usr/lib/x86_64-linux-gnu/pkgconfig")
pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)

add_executable(dfrobot_sen0358_read dfrobot_sen0358_read.cc)
target_link_libraries(dfrobot_sen0358_read PkgConfig::MODBUS)


