#include <sys/cdefs.h>/*
This example sets D8/D9 as pin mode to UART and then opens UART1 device.
It sets UART settings such as baud rate, stop bits, and register read callback.
When there are some data available to the device through UART1.
It reads data with vm_dcl_read and then writes same data through UART1 with vm_dcl_write.
*/
#include "../include/vmtype.h"
#include "../include/vmlog.h"
#include "../include/vmsystem.h"
#include "../include/vmdatetime.h"
#include "../include/vmtimer.h"
#include "../include/vmdcl.h"
#include "../include/vmdcl_gpio.h"
#include "../include/vmdcl_sio.h"
#include "../include/vmfs.h"
#include "../include/vmchset.h"
#include "blerssi.h"
#include <stdio.h>
#include <string.h>

//  TODO: Only for Mac.
#ifndef __unused
#define __unused
#endif  //  __unused

//  LED to GPIO mapping, according to https://github.com/loboris/RePhone_on_Linux/blob/master/Documents/Lua%20on%20RePhone%20Manual.pdf
const int MAX_LED = 3;
const unsigned int GPIO_LED[3] = { 17 /* red */, 15 /* green */, 12 /* blue */ };
VM_DCL_HANDLE gpio_handle[3] = {0, 0, 0};  //  Handle for each GPIO LED.

VM_TIMER_ID_PRECISE  __unused sys_timer_id = 0;
VM_DCL_HANDLE g_uart_handle = 0;  /* handle of UART */
VM_DCL_OWNER_ID g_owner_id = 0;  /* Module owner of APP */
vm_dcl_sio_control_dcb_t g_config;  /* UART configuration setting */
VMCHAR filename[VM_FS_MAX_PATH_LENGTH] = {0};
VMWCHAR wfilename[VM_FS_MAX_PATH_LENGTH] = {0};
static char buffer[256];
static char buffer2[256];
int led_color = 2;  //  Blue is default LED color.

void log_to_file(const char *log)
{
    //  Write the log to the log file c:\log.txt and debug console.
    if (log <= 0) return;
    vm_date_time_t time;
    VMINT ret = vm_time_get_date_time(&time);
    if (ret >= 0) {  //  Add the date time to the log.
        snprintf(buffer2, sizeof(buffer2), "%04d-%02d-%02d %02d:%02d:%02d: %s\n", time.year, time.month, time.day,
                 time.hour, time.minute, time.second, log);
    }
    else {
        snprintf(buffer2, sizeof(buffer2), "%s\n", log);
    }
    VMUINT writelen = 0;
    vm_log_info(log);
    if (filename[0] == 0) {  //  Compose the filename c:\log.txt.
        snprintf((char *) filename, VM_FS_MAX_PATH_LENGTH, "%c:\\%s", vm_fs_get_internal_drive_letter(), "log.txt");
        vm_chset_ascii_to_ucs2(wfilename, VM_FS_MAX_PATH_LENGTH, filename);
    }
    //  Create file.
    VM_FS_HANDLE filehandle = vm_fs_open(wfilename, VM_FS_MODE_APPEND, TRUE);
    if (filehandle < 0) {
        vm_log_info("log_to_file: Failed to create file: %s",filename);
        return;
    }
    //  Write file.
    ret = vm_fs_write(filehandle, (void*)buffer2, strlen(buffer2), &writelen);
    if (ret < 0) {
        vm_log_info("log_to_file: Failed to write file");
        return;
    }
    //  Close file.
    vm_fs_close(filehandle);
}

void gpio_init(void)
{
    //  Init the GPIO for each LED (red. green, blue).
    for (int i = 0; i < MAX_LED; i++) {
        gpio_handle[i] = vm_dcl_open(VM_DCL_GPIO, GPIO_LED[i]);
        vm_dcl_control(gpio_handle[i], VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
        vm_dcl_control(gpio_handle[i], VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
    }
}

VMCHAR last_response[256] = {0};

void uart_irq_handler(void* __unused parameter, VM_DCL_EVENT event, VM_DCL_HANDLE device_handle){
    //  Handle received UART data.
    if(event == VM_DCL_SIO_UART_READY_TO_READ){
        //  Data has arrived.
        VM_DCL_STATUS status;
        VM_DCL_BUFFER_LENGTH returned_len;
        //  Read data into buffer.
        status = vm_dcl_read(device_handle,(VM_DCL_BUFFER*)last_response, sizeof(last_response), &returned_len, (unsigned int) vm_dcl_get_owner_id());
        if(status < VM_DCL_STATUS_OK){
            log_to_file("uart_irq_handler: read failed");
            return;
        }
        led_color = 0;  //  Blink Red when data received.
        if (returned_len >= 0) {
            last_response[returned_len] = 0;
            snprintf(buffer, sizeof(buffer), "uart_irq_handler: read length = %d | %s", (int) returned_len, (char *) last_response); log_to_file(buffer);
        }
        else {
            snprintf(buffer, sizeof(buffer), "uart_irq_handler: ERROR read length = %d", (int) returned_len); log_to_file(buffer);
        }
    }
}

void send_uart_data(unsigned char data[], unsigned int len) {
    //  Send data to UART port e.g. "AT$SS=31323334\r".
    snprintf(buffer, sizeof(buffer), "send_uart_data: length = %d | %s", len, (char *) data); log_to_file(buffer);
    if(g_uart_handle != -1){
        //  Write data
        VM_DCL_BUFFER_LENGTH write_len = 0;
        VM_DCL_STATUS status = vm_dcl_write(g_uart_handle, data, len, &write_len, (unsigned int) vm_dcl_get_owner_id());
        //  Continue to write data if write fails.
        VMINT count = 0;
        while((status < VM_DCL_STATUS_OK || write_len != len) && (count < 3))
        {
            count++;
            status = vm_dcl_write(g_uart_handle, data, len, &write_len, (unsigned int) vm_dcl_get_owner_id());
        }
        snprintf(buffer, sizeof(buffer), "send_uart_data: write length = %d", (int) write_len); log_to_file(buffer);
    }
    else {
        log_to_file("send_uart_data: UART not opened yet");
    }
}

//  Sample command to be sent.
const char no_echo_cmd[] = "ATE0\r";  //  Suppress echo.
const char transmit_cmd[] = "AT$SS=31323334\r";  //  Transmit data bytes, up to 12 bytes.

void sys_timer_callback(VM_TIMER_ID_PRECISE  __unused sys_timer_id, void __unused * user_data)
{
    //  This call back happens every 5 seconds.  We flash blue if no data received, else we flash red.
    static int out = 0;
    static int last_led_color = 0;
    out = 1 - out;
    if (out) {
        last_led_color = led_color;
        led_color = 2;  //  Blue is default color of LED.
        vm_dcl_control(gpio_handle[last_led_color], VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
    } else {
        vm_dcl_control(gpio_handle[last_led_color], VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
    }
    log_to_file(out ? "sys_timer_callback: led on" : "sys_timer_callback: led off");
    //  Send a transmit command.
    const char *cmd = transmit_cmd;
    //  If we are getting AT echo, then turn echo off.
    if (last_response[0] == 'A' && last_response[1] == 'T')
        cmd = no_echo_cmd;
    send_uart_data((unsigned char *) cmd, strlen(cmd));
}

void handle_sysevt(VMINT message, VMINT __unused param) {
    //  The callback to be invoked by the system engine.
    switch (message){
        case VM_EVENT_CREATE:
            //  At startup, init LED and register a callback every 5 seconds.
            gpio_init();
            sys_timer_id = vm_timer_create_precise(5 * 1000, sys_timer_callback, NULL);
            //  Init UART1.
            vm_dcl_config_pin_mode(VM_PIN_P8, VM_DCL_PIN_MODE_UART);
            vm_dcl_config_pin_mode(VM_PIN_P9, VM_DCL_PIN_MODE_UART);
            g_owner_id = vm_dcl_get_owner_id();
            //  Open UART1.
            g_uart_handle = vm_dcl_open(VM_DCL_SIO_UART_PORT1, (unsigned int) vm_dcl_get_owner_id());
            if(VM_DCL_HANDLE_INVALID == g_uart_handle){
                log_to_file("handle_sysevt: UART open failed");
            }
            else{
                g_config.owner_id = (unsigned int) g_owner_id;
                g_config.config.dsr_check = 0;
                g_config.config.data_bits_per_char_length = VM_DCL_SIO_UART_BITS_PER_CHAR_LENGTH_8;
                g_config.config.flow_control = VM_DCL_SIO_UART_FLOW_CONTROL_NONE;
                g_config.config.parity = VM_DCL_SIO_UART_PARITY_NONE;
                g_config.config.stop_bits = VM_DCL_SIO_UART_STOP_BITS_1;
                g_config.config.baud_rate = VM_DCL_SIO_UART_BAUDRATE_9600;
                g_config.config.sw_xoff_char = 0x13;
                g_config.config.sw_xon_char = 0x11;
                //  Configure UART1 for 9600 baud.
                vm_dcl_control(g_uart_handle, VM_DCL_SIO_COMMAND_SET_DCB_CONFIG,(void *)&g_config);
                //  Resgiter callback when data received.
                vm_dcl_register_callback(g_uart_handle, VM_DCL_SIO_UART_READY_TO_READ, (vm_dcl_callback)uart_irq_handler, NULL);
            }
            log_to_file("handle_sysevt: Initialised");
            break;
        case VM_EVENT_QUIT:
            vm_dcl_close(g_uart_handle);
            break;
        default:
            break;
    }
    blerssi_handle_sysevt(message, param);
}

void test_log()
{
    //  Write the log to the log file c:\log.txt and debug console.
    VMCHAR filename2[VM_FS_MAX_PATH_LENGTH] = {0};
    VMWCHAR wfilename2[VM_FS_MAX_PATH_LENGTH] = {0};
    VMUINT writelen = 0;
    strcpy((char *) filename2, "c:\\log1.txt");
    //snprintf((char *) filename, VM_FS_MAX_PATH_LENGTH, "%c:\\%s", vm_fs_get_internal_drive_letter(), "log.txt");
    vm_chset_ascii_to_ucs2(wfilename2, VM_FS_MAX_PATH_LENGTH, filename2);
    //  Create file.
    VM_FS_HANDLE filehandle = vm_fs_open(wfilename2, VM_FS_MODE_APPEND, FALSE);
    if (filehandle < 0) return;
    //  Write file.
    VM_RESULT ret = vm_fs_write(filehandle, "test\n", 5, &writelen);
    if (ret < 0) return;
    //  Close file.
    vm_fs_close(filehandle);
}

void __unused vm_main(void){
    test_log();
    //  This is the entry point for the program. We register the handlers for system events.
    vm_pmng_register_system_event_callback(handle_sysevt);
}

