cmake_minimum_required(VERSION 2.8)
project(carryobjects)

# fetch environment variables
set(PLAYER_INCLUDE_DIRS $ENV{PLAYER_INCLUDE_DIRS})
set(PLAYER_LIBRARY_PATH $ENV{PLAYER_LIBRARY_PATH})

# find player library
find_library(PLAYER_LIBRARY playerc++ ${PLAYER_LIBRARY_PATH})
find_path(PLAYER_DIRECTORY playerc++ ${PLAYER_LIBRARY_PATH})
include_directories(${PLAYER_INCLUDE_DIRS})

# define our own executable
add_executable(carryobjects main.cpp robot.cpp)
target_link_libraries(carryobjects ${PLAYER_LIBRARY})
