#include <iostream>
#include <string>

#include <i2clib/PCA9685.hpp>

using namespace std;
using namespace i2clib;

static constexpr int ARGC_MIN = 4;
static constexpr int ARG_INDEX_CMD = 3;

void usage(string const& cmd, ostream& io) {
    io
        << "usage: " << cmd << " DEV ADDRESS CMD [ARGS]\n"
        << "  set-period TIME_NS PWM period in nanoseconds (must sleep first)"
        << "  sleep put the chip to sleep"
        << "  wakeup wake the chip up after a sleep"
        << "  set-duty DUTY_CYCLE where DUTY_CYCLE is a float between 0 and 1"
        << endl;
}

void validateCmdArgc(string cmd, int argc, int expectedCmdArgs) {
    if (argc < expectedCmdArgs + ARGC_MIN) {
        cerr << "not enough arguments to " << cmd << endl;
        exit(1);
    }
    else if (argc > expectedCmdArgs + ARGC_MIN) {
        cerr << "too many arguments to " << cmd << endl;
        exit(1);
    }
}

int main(int argc, char** argv)
{
    if (argc == 1) {
        usage(argv[0], cout);
        return 0;
    }
    else if (argc < ARGC_MIN) {
        usage(argv[0], cerr);
        return 1;
    }

    string i2c_dev = argv[1];
    int address = stoi(argv[2]);
    string cmd = argv[ARG_INDEX_CMD];

    I2CBus bus(i2c_dev);

    if (cmd == "sleep") {
        i2clib::PCA9685 chip(bus, address);
        chip.writeSleepMode();
    }
    else if (cmd == "wakeup") {
        i2clib::PCA9685 chip(bus, address);
        chip.writeNormalMode();
    }
    else if (cmd == "set-period") {
        validateCmdArgc("set-period", argc, 1);
        auto duration = stoi(argv[ARG_INDEX_CMD + 1]);

        i2clib::PCA9685 chip(bus, address);
        chip.writeCycleDuration(duration);
    }
    else if (cmd == "set-duty") {
        validateCmdArgc("set-duty", argc, 2);
        auto pwm = stoi(argv[ARG_INDEX_CMD + 1]);
        auto ratio = stof(argv[ARG_INDEX_CMD + 2]);

        i2clib::PCA9685 chip(bus, address);
        chip.writeDutyCycles(pwm, { ratio });
    }
    return 0;
}
