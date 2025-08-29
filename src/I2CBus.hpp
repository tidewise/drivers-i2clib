#ifndef I2CLIB_I2CBUS_HPP
#define I2CLIB_I2CBUS_HPP

#include <base/Time.hpp>

#include <array>
#include <string>

namespace i2clib {
    /** Class to access an i2c bus
     *
     * Note that concurrent access is handled by the Linux kernel. It is fine to have
     * more than one I2CBus object access the same bus, both in the same process and in
     * different processes
     */
    class I2CBus {
        int m_fd = -1;

        base::Time m_timeout = base::Time::fromMilliseconds(100);

    public:
        I2CBus(std::string const& path);
        ~I2CBus();

        /** Configure the i2c timeout
         *
         * The default is 100ms, it is enforced on construction
         */
        void setTimeout(base::Time const& timeout);

        /** Write \c size bytes at the given address
         */
        void write(uint8_t address, uint8_t* bytes, size_t size);

        /** @overload compatible with initializer lists
         *
         * @example i2c.write(DEVICE_ADRESS, { 1, 2 })
         */
        template <int Size>
        void write(uint8_t address, uint8_t const (&data)[Size])
        {
            std::array<uint8_t, Size> rw_data;
            std::copy(data, data + Size, rw_data.begin());
            return write(address, rw_data.data(), Size);
        }
    };
}

#endif