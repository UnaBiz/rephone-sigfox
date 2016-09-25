cmake_minimum_required(VERSION 3.6)
project(run-uart)
set(SOURCE_FILES
        main.c
        blerssi.c
        Makefile
        run-uart.project)

##################################################

set(CMAKE_VERBOSE_MAKEFILE TRUE)
add_definitions(-D__HDK_LINKIT_ASSIST_2502__ -D__COMPILER_GCC__)

set(WORKSPACE_PATH ${CMAKE_SOURCE_DIR}/)
set(PROJECT_PATH ${WORKSPACE_PATH}/${PROJECT_NAME}/)
set(LINKIT_ASSIST_SDK_PATH ${WORKSPACE_PATH}LINKIT_ASSIST_SDK/)

file(GLOB_RECURSE USER_SOURCES "${PROJECT_PATH}/*.c")
file(GLOB_RECURSE COMMON_SOURCES "${WORKSPACE_PATH}common/*.o")

set(OBJECTS "${WORKSPACE_PATH}common/lcd_sitronix_st7789s.o ${WORKSPACE_PATH}common/tp_goodix_gt9xx.o ${WORKSPACE_PATH}common/tp_i2c.o")
set(SYS_OBJECTS ${WORKSPACE_PATH}common/gccmain.o)

include_directories(${WORKSPACE_PATH}include)
include_directories(${WORKSPACE_PATH}common)

set(LINKER_SCRIPT "${WORKSPACE_PATH}lib/LINKIT10/armgcc/scat.ld")
link_directories(${WORKSPACE_PATH}lib)
#link_libraries(${WORKSPACE_PATH}lib/LINKIT10/armgcc/percommon.a -lm)
set(LIBRARIES "${WORKSPACE_PATH}lib/LINKIT10/armgcc/percommon.a -lm")

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${COMMON_SOURCES} ${LINKER_SCRIPT} ${WORKSPACE_PATH}common/lcd_sitronix_st7789s ${WORKSPACE_PATH}common/tp_goodix_gt9xx ${WORKSPACE_PATH}common/tp_i2c)
target_link_libraries(${PROJECT_NAME}.elf)

###############################################################################
##set(GCC_BIN "/eclipse/LINKIT_ASSIST_SDK/tools/gcc-arm-none-eabi-4_9-2014q4-20141203-win32/bin/")  ##  Windows
set(AS      ${GCC_BIN}arm-none-eabi-as)
set(CC      ${GCC_BIN}arm-none-eabi-gcc)
set(CPP     ${GCC_BIN}arm-none-eabi-g++)
set(LD      ${GCC_BIN}arm-none-eabi-ld)
set(OBJCOPY ${GCC_BIN}arm-none-eabi-objcopy)
set(OBJDUMP ${GCC_BIN}arm-none-eabi-objdump)
set(SIZE    ${GCC_BIN}arm-none-eabi-size)
set(AR      ${GCC_BIN}arm-none-eabi-ar)

set(PACK    python2.7 ${WORKSPACE_PATH}tools/packtag.py)  ##  MacOS.
set(PACK    python ${WORKSPACE_PATH}tools/packtag.py)
set(PUSH    "@echo set RePhone into storage mode, put ${PROJECT_NAME}.vxp into MRE directory and replace vxp file name with ${PROJECT_NAME}.vxp")

set(CPU "-mcpu=arm7tdmi-s -mthumb -mlittle-endian")
set(CC_FLAGS "${CPU} -fvisibility=hidden -fpic -O2")

set(LD_FLAGS "-O2 -Wl,--gc-sections --specs=nosys.specs -fpic -pie -Wl,-Map=${PROJECT}.map  -Wl,--entry=gcc_entry -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-unresolved-symbols")
set(LD_SYS_LIBS "")

set(CMAKE_CXX_FLAGS "${CC_FLAGS} ${CC_SYMBOLS} -c -std=c++11")
set(CMAKE_C_FLAGS "${CC_FLAGS} ${CC_SYMBOLS} -c -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS ${LD_FLAGS})

################################################################
# specify the cross compiler
set(CMAKE_AR ${AR})
set(CMAKE_C_COMPILER ${CC})
set(CMAKE_CXX_COMPILER ${CPP})

set(ALL_OBJECTS "${PROJECT_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}.elf.dir/${PROJECT_NAME}/*.obj")

set(CMAKE_C_LINK_EXECUTABLE "${CC} ${LD_FLAGS} -T${LINKER_SCRIPT} ${LIBRARY_PATHS} -o ${PROJECT_NAME}.elf -Wl,--start-group ${LD_SYS_LIBS} ${ALL_OBJECTS} ${SYS_OBJECTS} ${OBJECTS} ${LIBRARIES} -Wl,--end-group")
set(CMAKE_CXX_LINK_EXECUTABLE "${CPP} ${LD_FLAGS} -T${LINKER_SCRIPT} ${LIBRARY_PATHS} -o ${PROJECT_NAME}.elf -Wl,--start-group ${LD_SYS_LIBS} ${ALL_OBJECTS} ${SYS_OBJECTS} ${OBJECTS} ${LIBRARIES} -Wl,--end-group")

# Convert .elf to .vxp
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${OBJCOPY} --strip-debug ${PROJECT_NAME}.elf
        COMMAND ${PACK} ${PROJECT_NAME}.elf ${PROJECT_NAME}.vxp
        COMMAND copy /y ${PROJECT_NAME}.vxp F:\\MRE\\${PROJECT_NAME}.vxp && echo "********************** Copied VXP to device"
)

#add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
#        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
#        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
#        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")

#add_custom_command(OUTPUT $(PROJECT_NAME).elf
#    COMMAND $(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ -Wl,--start-group $^ $(LIBRARIES) $(LD_SYS_LIBS) -Wl,--end-group
#)

