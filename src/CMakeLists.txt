rock_library(i2clib
    SOURCES I2CBus.cpp PCA9685.cpp
    HEADERS I2CBus.hpp PCA9685.hpp
    DEPS_PKGCONFIG base-types
)

rock_executable(
    i2c_pca9685_ctl PCA9685Main.cpp
    DEPS i2clib
)