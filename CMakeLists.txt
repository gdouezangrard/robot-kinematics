project(client)
cmake_minimum_required(VERSION 2.6)

set(VREP_PATH "${VREP_ROOT}/programming" CACHE STRING "V-REP API")

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wno-reorder -pedantic")

add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

include_directories(
	src
	lib
	${VREP_PATH}/include
	${VREP_PATH}/remoteApi)

set(SOURCES
	lib/VREPClient.cpp lib/Motor.cpp lib/Object.cpp lib/ForceSensor.cpp
	${VREP_PATH}/remoteApi/extApi.c ${VREP_PATH}/remoteApi/extApiPlatform.c)
set(EXTRA_LIBRARIES pthread ncurses)

add_executable(${PROJECT_NAME} ${SOURCES} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBRARIES})
