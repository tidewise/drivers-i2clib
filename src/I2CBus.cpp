#include <i2clib/Exceptions.hpp>
#include <i2clib/I2CBus.hpp>

#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace i2clib;
using namespace std;

I2CBus::I2CBus(std::string const& path)
{
    int fd = open(path.c_str(), O_RDWR);
    if (fd == -1) {
        throw IOError(string("failed to open bus: ") + strerror(errno));
    }
    m_fd = fd;

    setTimeout(m_timeout);
}

I2CBus::~I2CBus()
{
    close(m_fd);
}

void I2CBus::setTimeout(base::Time const& timeout)
{
    int ret = ioctl(m_fd,
        I2C_TIMEOUT,
        static_cast<unsigned long>(timeout.toMilliseconds() / 10));
    if (ret == -1) {
        throw IOError("could not configure i2c bus timeout");
    }
}

void I2CBus::write(uint8_t address, uint8_t* registers, size_t size)
{
    i2c_msg config_msg;
    config_msg.flags = 0;
    config_msg.addr = address;
    config_msg.len = size;
    config_msg.buf = registers;

    i2c_rdwr_ioctl_data query;
    query.msgs = &config_msg;
    query.nmsgs = 1;
    if (ioctl(m_fd, I2C_RDWR, &query) == -1) {
        ostringstream message;
        message << "failed write to address " + to_string(address) << ": ";
        message << strerror(errno);
        for (size_t i = 0; i < size; ++i) {
            message << " " << hex << static_cast<int>(registers[i]);
        }
        throw WriteError(message.str());
    }
}