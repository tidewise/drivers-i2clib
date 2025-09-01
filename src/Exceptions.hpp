#ifndef I2CLIB_EXCEPTIONS_HPP
#define I2CLIB_EXCEPTIONS_HPP

#include <stdexcept>

namespace i2clib {
    struct IOError : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };
    struct ReadError : public IOError {
        using IOError::IOError;
    };
    struct WriteError : public IOError {
        using IOError::IOError;
    };
}

#endif