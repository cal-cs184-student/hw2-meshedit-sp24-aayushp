# Install script for directory: /Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src

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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/cmake-build-debug/CGL/src/libCGL.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
    execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCGL.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/CGL.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/vector2D.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/vector3D.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/vector4D.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/matrix3x3.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/matrix4x4.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/quaternion.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/complex.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/color.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/osdtext.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/viewer.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/base64.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/tinyxml2.h"
    "/Users/aayushpatel/Desktop/CS_184/hw2-meshedit-sp24-aayushp/CGL/src/renderer.h"
    )
endif()

