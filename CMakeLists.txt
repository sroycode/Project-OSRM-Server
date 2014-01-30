cmake_minimum_required(VERSION 2.6)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(OSRM)
include(FindPackageHandleStandardArgs)
set(HUGO "${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(GetGitRevisionDescription)
git_describe(GIT_DESCRIPTION)

TRY_RUN(SHARED_LIBRARY_PATH_TYPE SHARED_LIBRARY_PATH_INFO_COMPILED ${PROJECT_BINARY_DIR}/CMakeTmp ${PROJECT_SOURCE_DIR}/cmake/size.cpp OUTPUT_VARIABLE IS_64_SYSTEM)
if(IS_64_SYSTEM)
	message(STATUS "System supports 64 bits.")
	set( HAS64BITS 1 )
else(IS_64_SYSTEM)
	MESSAGE(WARNING "Compiling on a 32 bit system is unsupported!")
	set( HAS64BITS 0 )
endif(IS_64_SYSTEM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/Util/UUID.cpp UUID.cpp.alwaysbuild
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/UUID-Config.cmake
  DEPENDS
    ${CMAKE_SOURCE_DIR}/Util/UUID.cpp.in
  	${CMAKE_SOURCE_DIR}/cmake/UUID-Config.cmake
  COMMENT "Configuring UUID.cpp"
  VERBATIM)

add_custom_target(UUIDConfigure DEPENDS ${CMAKE_SOURCE_DIR}/Util/UUID.cpp )

set(BOOST_COMPONENTS filesystem program_options regex system thread)

configure_file(Util/GitDescription.cpp.in ${CMAKE_SOURCE_DIR}/Util/GitDescription.cpp)

add_executable(osrm-routed routed.cpp Util/GitDescription.cpp)
set_target_properties(osrm-routed PROPERTIES COMPILE_FLAGS -DROUTED)

file(GLOB DescriptorGlob Descriptors/*.cpp)
file(GLOB LibOSRMGlob Library/*.cpp)
file(GLOB SearchEngineSource DataStructures/SearchEngine*.cpp)
file(GLOB ServerStructureGlob Server/DataStructures/*.cpp)

set(OSRMSources ${LibOSRMGlob} ${DescriptorGlob} ${SearchEngineSource} ${ServerStructureGlob})
add_library(OSRM SHARED ${OSRMSources})
add_library(UUID STATIC Util/UUID.cpp)
add_dependencies( UUID UUIDConfigure )

# Check the release mode
if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
	set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE MATCHES Debug)
if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS "Configuring OSRM in debug mode")
endif(CMAKE_BUILD_TYPE MATCHES Debug)
if(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "Configuring OSRM in release mode")
endif(CMAKE_BUILD_TYPE MATCHES Release)

#Configuring compilers
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# using Clang
	set(CMAKE_CXX_FLAGS "-Wall -Wno-unknown-pragmas -Wno-unneeded-internal-declaration")
	message(STATUS "OpenMP parallelization not available using clang++")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	# using GCC
	set(CMAKE_CXX_FLAGS "-Wall -fopenmp -pedantic")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	# using Intel C++
	set(CMAKE_CXX_FLAGS "-static-intel -wd10237 -Wall -openmp -ipo")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	# using Visual Studio C++
endif()

if(APPLE)
	SET(CMAKE_OSX_ARCHITECTURES "x86_64")
	message("Set Architecture to x64 on OS X")
endif()

#Check Boost
set(BOOST_MIN_VERSION "1.44.0")
find_package( Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} REQUIRED )
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost (version >= 1.44.0) required.\n")
endif (NOT Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})

IF( APPLE )
	target_link_libraries( OSRM ${Boost_LIBRARIES} UUID )
ELSE( APPLE )
	target_link_libraries( OSRM ${Boost_LIBRARIES} )
ENDIF( APPLE )
target_link_libraries( osrm-routed ${Boost_LIBRARIES} OSRM UUID )

find_package ( BZip2 REQUIRED )
include_directories(${BZIP_INCLUDE_DIRS})

find_package( ZLIB REQUIRED )
include_directories(${ZLIB_INCLUDE_DIRS})
target_link_libraries (osrm-routed ${ZLIB_LIBRARY})

find_package( Threads REQUIRED )

find_package( STXXL REQUIRED )
include_directories(${STXXL_INCLUDE_DIR})
target_link_libraries (OSRM ${STXXL_LIBRARY})

if(WITH_TOOLS)
	message("-- Activating OSRM internal tools")
	find_package( GDAL )
	if(GDAL_FOUND)
		add_executable(osrm-components Tools/componentAnalysis.cpp)
		include_directories(${GDAL_INCLUDE_DIR})
		target_link_libraries(
			osrm-components ${GDAL_LIBRARIES} ${Boost_LIBRARIES} UUID
		)
	endif(GDAL_FOUND)
	add_executable ( osrm-cli Tools/simpleclient.cpp Util/GitDescription.cpp)
	target_link_libraries( osrm-cli ${Boost_LIBRARIES} OSRM UUID )
    add_executable ( osrm-io-benchmark Tools/io-benchmark.cpp )
    target_link_libraries( osrm-io-benchmark ${Boost_LIBRARIES} )
endif(WITH_TOOLS)
