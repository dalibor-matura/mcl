cmake_minimum_required(VERSION 2.8)
project(fcl CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(FCLVersion)

option(FCL_STATIC_LIBRARY "Whether the FCL library should be static rather than shared" OFF)

option(FCL_64 "Whether the FCL library should be 64 bit" OFF)

# Whether to enable SSE
option(FCL_USE_SSE "Whether FCL should SSE instructions" ON)
set(FCL_HAVE_SSE 0)
if(FCL_USE_SSE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(FCL_HAVE_SSE 0) #always disable, for now
    add_definitions(-march=native)
  endif()
  # TODO: do something similar for other compilers
endif()

# Find Octomap (optional)
find_package(PkgConfig QUIET)
set(FCL_HAVE_OCTOMAP 0)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OCTOMAP QUIET octomap)
endif()
if(NOT OCTOMAP_FOUND)
    # if pkfconfig is not installed, then fall back on more fragile detection
    # of octomap
    find_path(OCTOMAP_INCLUDE_DIRS octomap.h
        PATH_SUFFIXES octomap)
    find_path(OCTOMAP_LIBRARY_DIRS
        ${CMAKE_SHARED_LIBRARY_PREFIX}octomap${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
        set(OCTOMAP_LIBRARIES "octomap;octomath")
    endif()
endif()
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
  include_directories(${OCTOMAP_INCLUDE_DIRS})
  link_directories(${OCTOMAP_LIBRARY_DIRS})
  set(FCL_HAVE_OCTOMAP 1)
  message(STATUS "FCL uses Octomap")
else()
  message(STATUS "FCL does not use Octomap")
endif()

find_package(Boost COMPONENTS thread date_time filesystem system unit_test_framework REQUIRED)

if(FCL_64)
	set(Boost_LIBRARYDIR "${Boost_LIBRARY_DIRS}/x64")
	set(Boost_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}/x64")
endif()

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARYDIR})

if(MSVC OR MSVC90 OR MSVC10)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)

include_directories("include")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(CCD REQUIRED ccd)
endif()
if(NOT CCD_FOUND)
    # if pkfconfig is not installed, then fall back on more fragile detection
    # of ccd
    find_path(CCD_INCLUDE_DIRS ccd.h
        PATH_SUFFIXES ccd)
    find_path(CCD_LIBRARY_DIRS
        ${CMAKE_SHARED_LIBRARY_PREFIX}ccd${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(CCD_INCLUDE_DIRS AND CCD_LIBRARY_DIRS)
        set(CCD_LIBRARIES "ccd")
    else()
        message(FATAL_ERROR "Libccd is required by FCL")
    endif()
endif()
include_directories(${CCD_INCLUDE_DIRS})
link_directories(${CCD_LIBRARY_DIRS})

#-------------------------------------------------------------------------------

#
# FCL's and Tests' common functions and variables
#

function(_FCL_CREATE_RELATIVE_PATHS _OUTPUT_LIBRARY _INPUT_LIBRARY)
  get_filename_component(
    _LIBRARY_RELATIVE_RELEASE
	"${${_INPUT_LIBRARY}_RELEASE}"
	NAME
  )
  
  get_filename_component(
    _LIBRARY_RELATIVE_DEBUG
	"${${_INPUT_LIBRARY}_DEBUG}"
	NAME
  )  
  
  set(${_OUTPUT_LIBRARY} "optimized;${_LIBRARY_RELATIVE_RELEASE};debug;${_LIBRARY_RELATIVE_DEBUG}" PARENT_SCOPE)
  
  # message(STATUS "_LIBRARY_RELATIVE_RELEASE = ${_LIBRARY_RELATIVE_RELEASE}")
  # message(STATUS "_LIBRARY_RELATIVE_DEBUG = ${_LIBRARY_RELATIVE_DEBUG}")
  # message(STATUS "_OUTPUT_LIBRARY = ${${_OUTPUT_LIBRARY}}")
endfunction()

_FCL_CREATE_RELATIVE_PATHS(Boost_SYSTEM_LIBRARY_RELATIVE_PATHS Boost_SYSTEM_LIBRARY)
_FCL_CREATE_RELATIVE_PATHS(Boost_THREAD_LIBRARY_RELATIVE_PATHS Boost_THREAD_LIBRARY)
_FCL_CREATE_RELATIVE_PATHS(Boost_DATE_TIME_LIBRARY_RELATIVE_PATHS Boost_DATE_TIME_LIBRARY)
_FCL_CREATE_RELATIVE_PATHS(Boost_UNIT_TEST_FRAMEWORK_LIBRARY_RELATIVE_PATHS Boost_UNIT_TEST_FRAMEWORK_LIBRARY)

#-------------------------------------------------------------------------------

add_subdirectory(include/fcl)
add_subdirectory(src)

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx"
  PATTERN ".DS_Store" EXCLUDE
  PATTERN ".svn" EXCLUDE
)

install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)


enable_testing()
add_subdirectory(test)
