# Install script for directory: /home/wrc/Desktop/streamer/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/simple_graph_model/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/vertical_layout/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/calculator/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/text/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/resizable_images/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/styles/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/connection_colors/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/dynamic_ports/cmake_install.cmake")
  include("/home/wrc/Desktop/streamer/cmake-build-debug/examples/lock_nodes_and_connections/cmake_install.cmake")

endif()

