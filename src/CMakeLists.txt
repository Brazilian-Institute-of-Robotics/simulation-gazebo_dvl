rock_library(gazebo_dvl
  SOURCES GazeboDVL.cpp ${msgs_SRCS}
  HEADERS GazeboDVL.hpp
  DEPS_PLAIN Boost
  DEPS_PKGCONFIG gazebo sdformat protobuf)
