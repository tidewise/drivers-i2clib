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
       << "  period-to-prescale PERIOD [FREQ] display the closest prescale parameters\n"
       << "    to achieve a given period. Pass the optional frequency parameter\n"
       << "    (in MHz) when using an external oscillator that is not 25MHz\n"
       << "  prescale-to-period FACTOR [FREQ] display the PWM period for a given\n"
       << "    prescale parameter. Pass the optional frequency parameter (in MHz)\n"
       << "    when using an external clock that is not 25MHz\n"
       << "  set-prescale FACTOR change PWM period by writing the prescale parameter\n"
       << "    The command stops PWM generation and put the chip to sleep\n"
       << "  restart perform the PWM generation restart after a sleep\n"
       << "  sleep put the chip to sleep\n"
       << "  enable-external-clock use an external clock instead of the internal one\n"
       << "    The command stops PWM generation and put the chip to sleep\n"
       << "  stop set all PWM outputs to zero\n"
       << "  wakeup wake the chip up after a sleep\n"
       << "  set-duty-us PWM TIME_US [FREQ] where PWM is the 0-based index of the PWM\n"
       << "    output whose duty cycle is being changed and TIME_US is the ON time in\n"
       << "    microseconds. Pass FREQ (in MHz) when using an external oscillator\n"
       << "    whose frequency is not 25MHz\n"
       << "  set-duty-ratio PWM RATIO where RATIO is the ratio of the ON phase of \n"
       << "    the PWM period, as a float between 0 and 1\n"
       << flush;
}

vector<string> validateCmdArgc(int argc, char** argv, int min, int max)
{
    if (argc < min + ARGC_MIN) {
        cerr << "not enough arguments to " << argv[ARG_INDEX_CMD] << endl;
        exit(1);
    }
    else if (argc > max + ARGC_MIN) {
        cerr << "too many arguments to " << argv[ARG_INDEX_CMD] << endl;
        exit(1);
    }

    return vector<string>(argv + ARGC_MIN, argv + argc);
}

vector<string> validateCmdArgc(int argc, char** argv, int expected)
{
    return validateCmdArgc(argc, argv, expected, expected);
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

    if (cmd == "prescale-to-period") {
        auto cmdArgs = validateCmdArgc(argc, argv, 1, 2);
        uint8_t prescale = stoi(cmdArgs[0]);
        float freq = PCA9685::INTERNAL_OSCILLATOR_FREQUENCY;
        if (cmdArgs.size() == 2) {
            freq = stof(cmdArgs[1]);
        }
        cout << PCA9685::prescaleToPeriod(prescale, freq) << " ns" << endl;
        return 0;
    }
    else if (cmd == "period-to-prescale") {
        auto cmdArgs = validateCmdArgc(argc, argv, 1, 2);
        uint32_t period = stoi(cmdArgs[0]);
        float freq = PCA9685::INTERNAL_OSCILLATOR_FREQUENCY;
        if (cmdArgs.size() == 2) {
            freq = stof(cmdArgs[1]);
        }

        uint8_t prescale = PCA9685::periodToPrescale(period, freq);
        if (prescale > 3) {
            cout << "prescale: " << prescale - 1 << ", "
                 << "period: " << PCA9685::prescaleToPeriod(prescale - 1, freq) << "\n";
        }
        cout << "prescale: " << prescale << ", "
             << "period: " << PCA9685::prescaleToPeriod(prescale, freq) << "\n";
        if (prescale < 255) {
            cout << "prescale: " << prescale + 1 << ", "
                 << "period: " << PCA9685::prescaleToPeriod(prescale + 1, freq) << "\n";
        }
        return 0;
    }

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
    else if (cmd == "set-prescale") {
        auto cmdArgs = validateCmdArgc(argc, argv, 1, 2);
        auto prescale = stoi(cmdArgs[0]);

        chip.stop();
        chip.writeSleepMode();
        chip.writePrescale(prescale);
    }
    else if (cmd == "set-duty-us") {
        auto cmdArgs = validateCmdArgc(argc, argv, 2, 3);
        auto pwm = stoi(cmdArgs[0]);
        uint32_t period = stoi(cmdArgs[1]);
        float freq = PCA9685::INTERNAL_OSCILLATOR_FREQUENCY;
        if (cmdArgs.size() == 3) {
            freq = stof(cmdArgs[2]);
        }

        chip.writeNormalMode();
        chip.writeDutyTimes(pwm, {period * 1000}, freq);
    }
    else if (cmd == "set-duty-ratio") {
        auto cmdArgs = validateCmdArgc(argc, argv, 2);
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
