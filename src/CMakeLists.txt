rock_library(gazebo_dvl
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(gazebo_dvl_bin Main.cpp
    DEPS gazebo_dvl)

