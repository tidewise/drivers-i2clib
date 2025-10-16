rock_library(i2clib
    SOURCES
        I2CBus.cpp
        PCA9685.cpp PCA9685PWMConfiguration.cpp
        BMP280.cpp
        MS5837.cpp
    HEADERS
        I2CBus.hpp
        PCA9685.hpp PCA9685PWMConfiguration.hpp
        BMP280.hpp BMP280Configuration.hpp BMP280Measurement.hpp
        MS5837.hpp MS5837Measurement.hpp
    DEPS_PKGCONFIG base-types
)

rock_executable(
    i2c_pca9685_ctl PCA9685Main.cpp
    DEPS i2clib
)

rock_executable(
    i2c_bmp280_ctl BMP280Main.cpp
    DEPS i2clib
)

rock_executable(
    i2c_ms5837_ctl MS5837Main.cpp
    DEPS i2clib
)