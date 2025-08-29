rock_library(i2clib
    SOURCES PCA9685.cpp
    HEADERS PCA9685.hpp)

rock_executable(
    i2c_pca9685_ctl PCA9685Main.cpp
    DEPS i2clib
)