#include "../include/vmlog.h"
#include "../include/vmsystem.h"
#include "../include/vmtimer.h"
#include "../include/vmdcl.h"
#include "../include/vmdcl_gpio.h"
#include "../include/vmfs.h"
#include <stdio.h>
#include <string.h>

VM_DCL_HANDLE gpio_handle;
VM_TIMER_ID_PRECISE sys_timer_id = 0;

void log_to_file(char *log)
{
    VMCHAR filename[VM_FS_MAX_PATH_LENGTH] = {0};
    VMWCHAR wfilename[VM_FS_MAX_PATH_LENGTH] = {0};
    VMWCHAR wfilename_copy[VM_FS_MAX_PATH_LENGTH] = {0};
    VM_FS_HANDLE filehandle = -1;
    VMUINT writelen = 0;
    VMINT ret = 0;
    VMCHAR filecontent[100] = {0};
    VMUINT readlen = 0;

    vm_log_info("FS_demo_create_and_copy_file -Start");

    sprintf(filename, "%c:\\%s", vm_fs_get_internal_drive_letter(), "log.txt");
    vm_chset_ascii_to_ucs2(wfilename, sizeof(wfilename), filename);

    /* create file */
    if ((filehandle = vm_fs_open(wfilename, VM_FS_MODE_APPEND, TRUE)) < 0)
    {
        vm_log_info("Failed to create file: %s",filename);
        return;
    }

    vm_log_info("Success to create file: %s", filename);

    /* write file */
    ret = vm_fs_write(filehandle, (void*)log, strlen(log), &writelen);
    if (ret < 0)
    {
        vm_log_info("Failed to write file");
        return;
    }
    vm_log_info("Success to write file: %s", filename);

    /* close file */
    vm_fs_close(filehandle);
}

void gpio_init(void)
{
    gpio_handle = vm_dcl_open(VM_DCL_GPIO, 12);     // blue
    vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
    vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
}

void sys_timer_callback(VM_TIMER_ID_PRECISE sys_timer_id, void* user_data)
{
    static int out = 0;
    
    out = 1 - out;
    if (out) {
        vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
    } else {
        vm_dcl_control(gpio_handle, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
    }
    
    vm_log_info("led %s", out ? "on" : "off");
}

void handle_sysevt(VMINT message, VMINT param)
{
    switch(message) {
    case VM_EVENT_CREATE:
        gpio_init();
        sys_timer_id = vm_timer_create_precise(1000, sys_timer_callback, NULL);
        break;
    case VM_EVENT_QUIT:
        break;
    }
}

/* Entry point */
void vm_main(void)
{
    /* register system events handler */
    vm_pmng_register_system_event_callback(handle_sysevt);
}
