cmake_minimum_required(VERSION 3.6)
project(rephone-sigfox)

include_directories(/usr/local/Cellar/arm-none-eabi-gcc/5.0-2016-q2-update/arm-none-eabi/include)
include_directories(include)

#add_executable(rephone-sigfox ${SOURCE_FILES})

include(run-uart/CMakeLists.txt)
