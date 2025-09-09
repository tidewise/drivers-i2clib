#ifndef I2CLIB_BMP280MEASUREMENT_HPP
#define I2CLIB_BMP280MEASUREMENT_HPP

#include <base/Time.hpp>
#include <base/Pressure.hpp>
#include <base/Temperature.hpp>

namespace i2clib {
    /** Compensated measurements from the BMP280
     */
    struct BMP280Measurement {
        base::Time time;
        base::Pressure pressure;
        base::Temperature temperature;
    };
}

#endif