#include <i2clib/MS5837.hpp>

#include <iostream>

using namespace i2clib;
using namespace std;
using namespace base;

static constexpr int ARGC_MIN = 4;
static constexpr int ARG_INDEX_CMD = 3;

void usage(string const& cmd, ostream& io)
{
    io << "usage: " << cmd << " DEV ADDRESS CMD [ARGS]\n"
       << "  NOTE: default address for this chip is 118\n"
       << "  prom: read PROM data\n"
       << "  raw: read raw data\n"
       << "  read: read and compute compensated data\n"
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

int main(int argc, char** argv) {
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

    i2clib::MS5837 chip(MS5837::MODEL_30BA, bus, address);
    if (cmd == "reset") {
        chip.reset();
    }
    else if (cmd == "prom") {
        auto prom = chip.readPROM();
        for (unsigned int i = 0; i < prom.C.size(); ++i) {
            std::cout << "C" << i << ": " << prom.C[i] << "\n";
        }
    }
    else if (cmd == "raw") {
        cout << "pressure: " << chip.readRawPressure(2) << "\n"
             << "temperature: " << chip.readRawTemperature(2) << "\n";
    }
    else if (cmd == "read") {
        auto meas = chip.read(2, 2);
        cout << meas.pressure.toBar() << " Bar, "
             << meas.temperature.getCelsius() << "C" << endl;
    }
    else {
        cerr << "Unknown command '" << cmd << "'" << endl;
        usage(argv[0], cerr);
        return 1;
    }
}