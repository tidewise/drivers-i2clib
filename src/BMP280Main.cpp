#include <i2clib/BMP280.hpp>

#include <iostream>

using namespace i2clib;
using namespace std;
using namespace base;

static constexpr int ARGC_MIN = 4;
static constexpr int ARG_INDEX_CMD = 3;

void usage(string const& cmd, ostream& io)
{
    io << "usage: " << cmd << " DEV ADDRESS CMD [ARGS]\n"
       << "  NOTE: default address for this chip is 0x76\n"
       << "  check: read and verify the chip ID\n"
       << "  normal: start periodic measurements\n"
       << "  sleep: stop measurements\n"
       << "  calibration: display calibration data\n"
       << "  raw: display raw data\n"
       << "  read: display compensated data\n"
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

    i2clib::BMP280 chip(bus, address);
    if (cmd == "check") {
        auto id = chip.readID();
        if (id != 0x58) {
            cerr << "Unexpected ID " << hex << id << ", expected 0x58" << endl;
        }
        else {
            cout << "OK" << endl;
        }
        return 0;
    }
    else if (cmd == "normal") {
        chip.writeMode(BMP280::MODE_NORMAL);
    }
    else if (cmd == "sleep") {
        chip.writeMode(BMP280::MODE_SLEEP);
    }
    else if (cmd == "defaults") {
        chip.sleepAndWriteConfiguration(BMP280Configuration());
    }
    else if (cmd == "calibration") {
        auto calibration = chip.readCalibration();
        std::cout
            << "T1: " << calibration.dig_T1 << "\n"
            << "T2: " << calibration.dig_T2 << "\n"
            << "T3: " << calibration.dig_T3 << "\n"
            << "P1: " << calibration.dig_P1 << "\n"
            << "P2: " << calibration.dig_P2 << "\n"
            << "P3: " << calibration.dig_P3 << "\n"
            << "P4: " << calibration.dig_P4 << "\n"
            << "P5: " << calibration.dig_P5 << "\n"
            << "P6: " << calibration.dig_P6 << "\n"
            << "P7: " << calibration.dig_P7 << "\n"
            << "P8: " << calibration.dig_P8 << "\n"
            << "P9: " << calibration.dig_P9 << "\n";
    }
    else if (cmd == "raw") {
        auto meas = chip.readRaw();
        cout << "pressure: " << hex << meas.pressure << ", "
             << "temperature: " << hex << meas.temperature << endl;
    }
    else if (cmd == "read") {
        auto meas = chip.read();
        cout << meas.pressure.toBar() << " Bar, "
             << meas.temperature.getCelsius() << "C" << endl;
    }
    else {
        cerr << "Unknown command '" << cmd << "'" << endl;
        usage(argv[0], cerr);
        return 1;
    }
}