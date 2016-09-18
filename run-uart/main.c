/*
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
#include "../include/vmboard.h"
#include <stdio.h>
#include <string.h>

VM_DCL_HANDLE gpio_handle;
VM_TIMER_ID_PRECISE sys_timer_id = 0;
VM_DCL_HANDLE g_uart_handle = 0;  /* handle of UART */
VM_DCL_OWNER_ID g_owner_id = 0;  /* Module owner of APP */
vm_dcl_sio_control_dcb_t g_config;  /* UART configuration setting */
VMCHAR filename[VM_FS_MAX_PATH_LENGTH] = {0};
VMWCHAR wfilename[VM_FS_MAX_PATH_LENGTH] = {0};
char buffer[256];
char buffer2[256];

void log_to_file(const char *log)
{
    //  Write the log to the log file c:\log.txt and debug console.
    if (log <= 0) return;
    vm_date_time_t time;
    VMINT ret = vm_time_get_date_time(&time);
    if (ret >= 0) {  //  Add the date time to the log.
        snprintf(buffer2, sizeof(buffer2), "%04d-%02d-%02d %02d:%02d:%02d: %s", time.year, time.month, time.day,
                 time.hour, time.minute, time.second, log);
    }
    else {
        snprintf(buffer2, sizeof(buffer2), "%s", log);
    }
    VM_FS_HANDLE filehandle = -1;
    VMUINT writelen = 0;
    vm_log_info(log);
    if (filename[0] == 0) {  //  Compose the filename c:\log.txt.
        snprintf(filename, sizeof(filename), "%c:\\%s", vm_fs_get_internal_drive_letter(), "log.txt");
        vm_chset_ascii_to_ucs2(wfilename, sizeof(wfilename), filename);
    }
    //  Create file.
    if ((filehandle = vm_fs_open(wfilename, VM_FS_MODE_APPEND, TRUE)) < 0) {
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
    //  Init the GPIO for the LED.
    gpio_handle = vm_dcl_open(VM_DCL_GPIO, 12);  //  Blue
    vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
    vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
}

void uart_irq_handler(void* parameter, VM_DCL_EVENT event, VM_DCL_HANDLE device_handle){
    //  Handle received UART data.
    if(event == VM_DCL_SIO_UART_READY_TO_READ){
        //  Data has arrived.
        VMCHAR data[256];
        VMINT i;
        VM_DCL_STATUS status;
        VM_DCL_BUFFER_LENGTH returned_len;
        VMINT count = 0;
        //  Read data into buffer.
        status = vm_dcl_read(device_handle,(VM_DCL_BUFFER*)data, sizeof(data), &returned_len, vm_dcl_get_owner_id());
        if(status < VM_DCL_STATUS_OK){
            log_to_file("uart_irq_handler: read failed");
            return;
        }
        snprintf(buffer, sizeof(buffer), "uart_irq_handler: read length = %d", returned_len); log_to_file(buffer);
        if (returned_len >= 0) buffer[returned_len] = 0;
        log_to_file(data);
    }
}

void send_uart_data(unsigned char data[], int len) {
    //  Send data to UART port.
    snprintf(buffer, sizeof(buffer), "send_uart_data: length = %d", len); log_to_file(buffer);
    log_to_file(data);
    if(g_uart_handle != -1){
        //  Write data
        VM_DCL_BUFFER_LENGTH write_len = 0;
        VM_DCL_STATUS status = vm_dcl_write(g_uart_handle, (VM_DCL_BUFFER*)data, len, &write_len, vm_dcl_get_owner_id());
        //  Continue to write data if write fails.
        VMINT count = 0;
        while((status < VM_DCL_STATUS_OK || write_len != len) && (count < 3))
        {
            count++;
            status = vm_dcl_write(g_uart_handle,(VM_DCL_BUFFER*)data, len, &write_len, vm_dcl_get_owner_id());
        }
        snprintf(buffer, sizeof(buffer), "send_uart_data: write length = %d", write_len); log_to_file(buffer);
    }
    else {
        log_to_file("send_uart_data: UART not opened yet");
    }
}

//  Sample command to be sent.
const char cmd[] = "AT\r";

void sys_timer_callback(VM_TIMER_ID_PRECISE sys_timer_id, void* user_data)
{
    //  This call back happens every 5 seconds.  We flash blue if no data received, else we flash red.
    static int out = 0;
    out = 1 - out;
    if (out) {
        vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
    } else {
        vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
    }
    log_to_file(out ? "sys_timer_callback: led on" : "sys_timer_callback: led off");
    //  Send a sample command.
    send_uart_data((unsigned char []) cmd, strlen(cmd));
}

void handle_sysevt(VMINT message, VMINT param) {
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
            g_uart_handle = vm_dcl_open(VM_DCL_SIO_UART_PORT1, vm_dcl_get_owner_id());
            if(VM_DCL_HANDLE_INVALID == g_uart_handle){
                log_to_file("handle_sysevt: UART open failed");
            }
            else{
                g_config.owner_id = g_owner_id;
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
                vm_dcl_register_callback(g_uart_handle, VM_DCL_SIO_UART_READY_TO_READ, (vm_dcl_callback)uart_irq_handler, (void*)NULL);
            }
            log_to_file("handle_sysevt: Initialised");
            break;
        case VM_EVENT_QUIT:
            vm_dcl_close(g_uart_handle);
            break;
    }
}

void vm_main(void){
    //  This is the entry point for the program. We register the handlers for system events.
    vm_pmng_register_system_event_callback(handle_sysevt);
}

