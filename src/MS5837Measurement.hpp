#ifndef I2CLIB_MS5837MEASUREMENT_HPP
#define I2CLIB_MS5837MEASUREMENT_HPP

#include <base/Time.hpp>
#include <base/Pressure.hpp>
#include <base/Temperature.hpp>

namespace i2clib {
    /** Measurement data from the MS5837
     */
    struct MS5837Measurement {
        base::Time time;
        base::Pressure pressure;
        base::Temperature temperature;
    };
}

#endif