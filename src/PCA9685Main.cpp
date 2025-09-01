#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <i2clib/PCA9685.hpp>

using namespace std;
using namespace i2clib;

static constexpr int ARGC_MIN = 4;
static constexpr int ARG_INDEX_CMD = 3;

void usage(string const& cmd, ostream& io)
{
    io << "usage: " << cmd << " DEV ADDRESS CMD [ARGS]\n"
       << "  set-period TIME_NS PWM period in nanoseconds\n"
       << "     The command stops PWM generation and put the chip to sleep\n"
       << "  restart perform the PWM generation restart after a sleep\n"
       << "  sleep put the chip to sleep\n"
       << "  enable-external-clock use an external clock instead of the internal one\n"
       << "     The command stops PWM generation and put the chip to sleep\n"
       << "  stop set all PWM outputs to zero\n"
       << "  wakeup wake the chip up after a sleep\n"
       << "  set-duty-us PWM TIME_US where TIME_US is the ON time in microseconds\n"
       << "  set-duty-ratio PWM RATIO where RATIO is the ratio of the ON phase of \n"
       << "       the PWM period, as a float between 0 and 1\n"
       << flush;
}

void validateCmdArgc(string cmd, int argc, int expectedCmdArgs)
{
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

    i2clib::PCA9685 chip(bus, address);
    if (cmd == "sleep") {
        chip.writeSleepMode();
    }
    else if (cmd == "stop") {
        chip.stop();
    }
    else if (cmd == "enable-external-clock") {
        chip.stop();
        chip.writeSleepMode();
        chip.enableExternalClock();
    }
    else if (cmd == "restart") {
        chip.writeNormalMode();
        this_thread::sleep_for(chrono::milliseconds(600));
        chip.writeRestart();
    }
    else if (cmd == "wakeup") {
        chip.writeNormalMode();
    }
    else if (cmd == "set-period") {
        validateCmdArgc("set-period", argc, 1);
        auto duration = stoi(argv[ARG_INDEX_CMD + 1]);

        chip.stop();
        chip.writeSleepMode();
        chip.writeCycleDuration(duration);
    }
    else if (cmd == "set-duty-us") {
        validateCmdArgc("set-duty-us", argc, 2);
        auto pwm = stoi(argv[ARG_INDEX_CMD + 1]);
        uint32_t period = stoi(argv[ARG_INDEX_CMD + 2]);

        chip.writeNormalMode();
        chip.writeDutyTimes(pwm, {period * 1000});
    }
    else if (cmd == "set-duty-ratio") {
        validateCmdArgc("set-duty", argc, 2);
        auto pwm = stoi(argv[ARG_INDEX_CMD + 1]);
        auto ratio = stof(argv[ARG_INDEX_CMD + 2]);

        chip.writeNormalMode();
        chip.writeDutyRatios(pwm, {ratio});
    }
    else {
        cerr << "invalid command " << cmd << endl;
        usage(argv[0], cerr);
        return 1;
    }
    return 0;
}
