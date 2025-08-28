rock_library(i2clib
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(i2clib_bin Main.cpp
    DEPS i2clib)

