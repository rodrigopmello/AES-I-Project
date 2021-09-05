cmake_minimum_required (VERSION 2.4)
project (smartdata)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Valid values: Release, Debug" FORCE)
endif(NOT CMAKE_BUILD_TYPE)

ADD_DEFINITIONS(-D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64)

if(CMAKE_BUILD_TYPE MATCHES Debug)
	ADD_DEFINITIONS(-D_DEBUG)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
else(CMAKE_BUILD_TYPE MATCHES Debug)
	message(FATAL_ERROR "Select build type")
endif(CMAKE_BUILD_TYPE MATCHES Debug)

include_directories (
	$(SMARTDATA_SOURCE_DIR).
	$(SMARTDATA_SOURCE_DIR)include
)

set(CMAKE_CXX_FLAGS_DEBUG "-m32 -Wall -O0 -pg -ggdb -Wfatal-errors")
set(CMAKE_CXX_FLAGS_RELEASE "-m32 -Wall -O3 -Wfatal-errors -Wuninitialized -fno-strict-aliasing -fomit-frame-pointer")

add_executable (smartdata
	src/main.cpp
	src/network/tstp/locator.cc
	src/network/tstp/manager.cc
	src/network/tstp/router.cc
	src/network/tstp/security.cc
	src/network/tstp/timekeeper.cc
	src/network/tstp/tstp.cc
	src/network/tstp/tstp_init.cc
	src/utility/aes.cc
	src/utility/bignum.cc
	src/utility/ostream.cc
	src/utility/random.cc
)

target_link_libraries (smartdata ${ADDITIONAL_LIBS} pthread rt) 
