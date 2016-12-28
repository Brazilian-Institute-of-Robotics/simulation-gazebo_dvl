#include <boost/test/unit_test.hpp>
#include <gazebo_dvl/Dummy.hpp>

using namespace gazebo_dvl;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    gazebo_dvl::DummyClass dummy;
    dummy.welcome();
}
