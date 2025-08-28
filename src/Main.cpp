#include <iostream>
#include <i2clib/Dummy.hpp>

int main(int argc, char** argv)
{
    i2clib::DummyClass dummyClass;
    dummyClass.welcome();

    return 0;
}
