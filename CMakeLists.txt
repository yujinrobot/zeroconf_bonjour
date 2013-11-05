##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(zeroconf_bonjour)

##############################################################################
# Apple Bonjour SDK
##############################################################################

set(Bonjour $ENV{BONJOUR_SDK_HOME})

if(NOT Bonjour)
  message(FATAL_ERROR "Install Apple Bonjour SDK for Windows firstly.")
endif()

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp zeroconf_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} "$ENV{BONJOUR_SDK_HOME}include")
link_directories(${catkin_LIBRARY_DIRS} "$ENV{BONJOUR_SDK_HOME}lib/win32")

##############################################################################
# Executable
##############################################################################

add_executable(zeroconf src/main.cpp src/bonjour.cpp src/messenger.cpp)
target_link_libraries(zeroconf ${catkin_LIBRARIES} dnssd.lib iphlpapi.lib ws2_32.lib)
install(TARGETS zeroconf 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
