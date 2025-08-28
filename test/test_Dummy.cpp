#include <boost/test/unit_test.hpp>
#include <i2clib/Dummy.hpp>

using namespace i2clib;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    i2clib::DummyClass dummy;
    dummy.welcome();
}
