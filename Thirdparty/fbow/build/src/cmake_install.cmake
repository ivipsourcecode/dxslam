# Install script for directory: /home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/src

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfbow.so.0.0.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfbow.so.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfbow.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/build/src/libfbow.so.0.0.1"
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/build/src/libfbow.so.0.0"
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/build/src/libfbow.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfbow.so.0.0.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfbow.so.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfbow.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/fbow" TYPE FILE FILES
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/src/fbow.h"
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/src/fbow_exports.h"
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/src/vocabulary_creator.h"
    "/home/gasper/longqiwei/dxslam/ivip/dxslam/Thirdparty/fbow/src/cpu.h"
    )
endif()

