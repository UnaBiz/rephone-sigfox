/*
  This example code is in public domain.

  This example code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

/*
  This example shows how to setup a BLE profile, specifically the BAS profile.
*/

#include "vmtype.h" 
#include "vmchset.h"
#include "vmstdlib.h"
#include "vmfs.h"
#include "vmlog.h" 
#include "vmwdt.h"
#include "vmcmd.h" 
#include "vmmemory.h"
//#include "ResID.h"
#include "BLERSSI.h"
#include "vmbt_gatt.h"
#include "vmlog.h"
#include "vmbt_cm.h"
#include "vmsystem.h"
#include "vmtimer.h"
#include "blerssi.h"
#include <stdio.h>
#include <string.h>

#define CATCHER_PORT  1000
/* Define this macro if the application supports running in the background. */
#define        SUPPORT_BG

#define IMMEDIATE_ALERT_SERVICE    0x1802   //Immediate Alert Service, 0x1802
#define ALERT_LEVEL_CHAR_UUID      0x2A06   //Alert Level Characteristic, 0x2A06
#define IAS_SERVICE_TYPE           0  		//Immediate Alert Service, 0x1802


typedef enum {
    APPC_STATUS_DISABLED = 0,
    APPC_STATUS_ENABLING,
    APPC_STATUS_ENABLED,
    APPC_STATUS_DISABLING,
}APPC_STATUS;

typedef struct {
    void                 *context_handle;
    APPC_STATUS            state;
    VMUINT8        uid[16];
}AppClientCntx;

typedef enum {
    APPC_STATUS_DISCONNECTED = 0,
    APPC_STATUS_CONNECTING,
    APPC_STATUS_CONNECTED,
    APPC_STATUS_DISCONNECTING,
}APP_CLIENT_CONNECTION_STATUS;

typedef struct {
    void                 *connection_handle;
    APP_CLIENT_CONNECTION_STATUS                   conn_status;
    VMCHAR                  bdaddr[VM_BT_GATT_ADDRESS_SIZE];
}AppClientConnCntx;


AppClientConnCntx *appc_conn_cntx = NULL;
AppClientCntx  g_appc_cntx = {0};

vm_bt_gatt_client_callback_t g_appc_cb;
vm_bt_gatt_address_t APP_CLIENT_BD_ADDR_LIST[10];
vm_bt_gatt_connection_t client_gatt_conn;
vm_bt_gatt_address_t            *client_bd_addr;

VMUINT8        g_appc_uid[] = {
    0xB4, 0x73, 0x1F, 0x49, 0xFF, 0xE5, 0x40, 0x56,
    0X84, 0x5B, 0x6D, 0xF1, 0xF1, 0xB1, 0x6E, 0x9D
};

VMUINT8 appc_srv_uuid[] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0X80,
    0x00, 0x10, 0x00, 0x00, 0x08, 0x18, 0x00, 0x10
};
VMUINT8 appc_char_uuid[] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0X80,
    0x00, 0x10, 0x00, 0x00, 0x09, 0x2A, 0x00, 0x10
};
vm_bt_gatt_attribute_value_t         char_value;

VMINT        layer_hdl[1];    /* layer handle array. */

int exiting = 0;

/****************************************************************************
 * FUNCTION DECLARATION
 ****************************************************************************/
static void app_client_check_bt_on_off(void);

static void app_client_bt_init_cb(VMUINT evt,void * param,void * user_data);

void app_client_init(void);
VMINT app_client_deinit(void);

static void app_client_register_client_callback(void *context_handle, VMBOOL status, VMUINT8 app_uuid[16]);

/* Callback for scan results */
static void app_client_vmt_scan_result_callback(void *context_handle, vm_bt_gatt_address_t *bd_addr, VMINT32 rssi, VMUINT8 eir_len, VMUINT8 *eir);

/*Callback indicating that a remote device has connected or has been disconnected */
static void app_client_connection_callback(vm_bt_gatt_connection_t *conn, VMBOOL connected, vm_bt_gatt_address_t *bd_addr);

/* Callback triggered in response to listen */
static void app_client_listen_callback(void *context_handle, VMBOOL status);

/*Callback triggered in response to set_adv_data */
static void app_client_set_adv_data_callback(void *context_handle, VMBOOL status){}

/**
 * Invoked in response to search_service when the GATT service search
 * has been completed.
 */
static void app_client_search_complete_callback(void *context_handle, VMBOOL status);

/*Reports GATT services on a remote device */
static void app_client_search_result_callback(vm_bt_gatt_connection_t *conn, vm_bt_gatt_service_info_t *uuid);

/* GATT characteristic enumeration result callback */
static void app_client_get_characteristic_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_characteristic_t *ch, VM_BT_GATT_CHAR_PROPERTIES properties);

/*GATT descriptor enumeration result callback */
static void app_client_get_descriptor_callback(vm_bt_gatt_connection_t *conn, VMBOOL status, vm_bt_gatt_client_descriptor_t *descr){}

/* GATT included service enumeration result callback */
static void app_client_get_included_service_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_service_info_t *svc_uuid, vm_bt_gatt_service_info_t *incl_svc_uuid){}

/* Callback invoked in response to [de]register_for_notification */
static void app_client_register_for_notification_callback(void *context_handle, VMBOOL status,
                                vm_bt_gatt_address_t *bd_addr, vm_bt_gatt_client_characteristic_t *ch){}

/*
 * Remote device notification callback, it is invoked when a remote device sends
 * a notification or indication that a client has registered.
 */
static void app_client_notify_callback(vm_bt_gatt_connection_t *conn, vm_bt_gatt_address_t *bd_addr,
                                vm_bt_gatt_client_characteristic_t *ch, vm_bt_gatt_attribute_value_t *value, VMBOOL is_notify){}

/* Reports result of a GATT read operation */
static void app_client_read_characteristic_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_characteristic_t *ch, vm_bt_gatt_attribute_value_t *value);

/* GATT write characteristic operation callback */
static void app_client_write_characteristic_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_characteristic_t *ch);

/* Callback invoked in response to read_descriptor */
static void app_client_read_descriptor_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_descriptor_t *descr, vm_bt_gatt_attribute_value_t *value){}

/* Callback invoked in response to write_descriptor */
static void app_client_write_descriptor_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_descriptor_t *descr){}

/* GATT executes a write callback */
static void app_client_execute_write_callback(vm_bt_gatt_connection_t *conn, VMBOOL status);

/* Callback triggered in response to read_remote_rssi */
static void app_client_read_remote_rssi_callback(void *context_handle, VMBOOL status, vm_bt_gatt_address_t *bd_addr, VMINT32 rssi);

/* Callback triggered in response to get_device_type */
static void app_client_get_device_type_callback(void *context_handle, VMBOOL status, vm_bt_gatt_address_t *bd_addr, VM_BT_GATT_CLIENT_DEV_TYPE dev_type){}

/****************************************************************************
 * local variables
 ****************************************************************************/
/*
* system events 
*/
void handle_sysevt(VMINT message, VMINT param);

/*
* key events
*/
void handle_keyevt(VMINT event, VMINT keycode);

/*
* pen events
*/
void handle_penevt(VMINT event, VMINT x, VMINT y);

/*
* demo
*/
static void draw_hello(void);

/****************************************************************************
 * content
 ****************************************************************************/

/*
* entry
*/
typedef struct
{
    vm_bt_gatt_connection_t conn;
    vm_bt_gatt_address_t bd_addr;
}RSSI_T;

void timer_cb(VM_TIMER_ID_NON_PRECISE tid, void* user_data)
{
    vm_log_debug("timer_cb user_data[0x%x]", user_data);
    ////  Kill the timer.
    vm_timer_delete_non_precise(tid);
    if (0x1 == user_data)
    {
        app_client_check_bt_on_off();
    }
    else if (0x2 == user_data)
	{
    	////  Restart.
    	exiting = 1;
    	vm_pmng_restart_application();
	}
    else
    {
        RSSI_T * rssi = (RSSI_T *)user_data;
        vm_log_debug("to read rssi[%x:%x:%x:%x:%x:%x]",
            rssi->bd_addr.data[5],
            rssi->bd_addr.data[4],
            rssi->bd_addr.data[3],
            rssi->bd_addr.data[2],
            rssi->bd_addr.data[1],
            rssi->bd_addr.data[0]
            );
        vm_bt_gatt_client_read_remote_rssi(rssi->conn.context_handle, &(rssi->bd_addr));
    }
}

void blerssi_handle_sysevt(VMINT message, VMINT param) {
    //  The callback to be invoked by the system engine.
    switch (message){
        case VM_EVENT_CREATE: {
            VM_TIMER_ID_NON_PRECISE timer_id = 0;
            timer_id = vm_timer_create_non_precise(5000, (vm_timer_non_precise_callback)timer_cb, 0x1);
            vm_log_debug("create timer [%d]", timer_id);
            break;
        }
        case VM_EVENT_QUIT:
        	app_client_deinit();
        	break;
        default:
            break;
    }
}

void old_vm_main(void) {
    //  This is the entry point for the program. We register the handlers for system events.
    vm_pmng_register_system_event_callback(blerssi_handle_sysevt);

    /* initialize layer handle */
    //layer_hdl[0] = -1;

    ///* register system events handler */
    //vm_reg_sysevt_callback(handle_sysevt);
    //
    ///* register keyboard events handler */
    //vm_reg_keyboard_callback(handle_keyevt);
    //
    ///* register pen events handler */
    //vm_reg_pen_callback(handle_penevt);

    /* Init MRE resource */
    //vm_res_init();
}

//void handle_sysevt(VMINT message, VMINT param) {
//#ifdef        SUPPORT_BG
///* The application updates the screen when receiving the message VM_MSG_PAINT
//*  that is sent after the application is activated. The application can skip
//*  the process on screen when the VM_MSG_ACTIVE or VM_MSG_INACTIVE is received.
//*
//    switch (message) {
//    case VM_MSG_CREATE:    /* the message of creation of application */
//        /* the GDI operation is not recommended as the response of the message*/
//        break;
//    case VM_MSG_PAINT:    /* the message of asking for application to repaint screen */
//        /* cerate base layer that has same size as the screen*/
//        layer_hdl[0] = vm_graphic_create_layer(0, 0,
//            vm_graphic_get_screen_width(),        /* get screen width */
//            vm_graphic_get_screen_height(),        /* get screen height */
//            -1);        /* -1 means layer or canvas have no transparent color */
//
//        /* set clip area */
//        vm_graphic_set_clip(0, 0,
//            vm_graphic_get_screen_width(),
//            vm_graphic_get_screen_height());
//
//        draw_hello();    /* draw hello world! */
//        //app_client_init();
//        app_client_check_bt_on_off();
//        break;
//    case VM_MSG_HIDE:
//        /* message of application state from foreground running to background running */
//        if( layer_hdl[0] != -1 )
//        {
//            vm_graphic_delete_layer(layer_hdl[0]);
//            layer_hdl[0] = -1;
//        }
//        break;
//    case VM_MSG_QUIT:    /* the message of quit of application */
//        if( layer_hdl[0] != -1 )
//        {
//            vm_graphic_delete_layer(layer_hdl[0]);
//            layer_hdl[0] = -1;
//        }
//
//        /* Release all resource */
//        vm_res_deinit();
//        app_client_deinit();
//
//        break;
//    }
//#else
//    switch (message) {
//    case VM_MSG_CREATE:    /* The message of the creation of application */
//    case VM_MSG_ACTIVE:    /* The message of the application when the state changes from inactive to active */
//        /*create base layer that has the same size as the screen*/
//        layer_hdl[0] = vm_graphic_create_layer(0, 0,
//            vm_graphic_get_screen_width(),        /* get screen width */
//            vm_graphic_get_screen_height(),        /* get screen height */
//            -1);        /* -1 means layer or canvas have no transparent color */
//
//        /* set clip area*/
//        vm_graphic_set_clip(0, 0,
//            vm_graphic_get_screen_width(),
//            vm_graphic_get_screen_height());
//        break;
//
//    case VM_MSG_PAINT:    /* the message of asking for application to repaint screen */
//        draw_hello();    /* draw hello world! */
//        //app_client_init();
//        app_client_check_bt_on_off();
//        break;
//
//    case VM_MSG_INACTIVE:    /* the message of application state from active to inactive */
//        if( layer_hdl[0] != -1 )
//            vm_graphic_delete_layer(layer_hdl[0]);
//
//        break;
//    case VM_MSG_QUIT:        /* the message of quit application */
//        if( layer_hdl[0] != -1 )
//            vm_graphic_delete_layer(layer_hdl[0]);
//
//        /* Release all resource */
//        vm_res_deinit();
//        app_client_deinit();
//        break;
//    }
//#endif
//}

void handle_keyevt(VMINT event, VMINT keycode) {
    /* press any key and return*/
#if 0
    if( layer_hdl[0] != -1 )
    {
        vm_graphic_delete_layer(layer_hdl[0]);
        layer_hdl[0] = -1;
    }

    vm_exit_app();        /* quit application */
#endif

}

void handle_penevt(VMINT event, VMINT x, VMINT y)
{
    #if 0
    /* touch and return*/
    if( layer_hdl[0] != -1 )
    {
        vm_graphic_delete_layer(layer_hdl[0]);
        layer_hdl[0] = -1;
    }

    vm_exit_app();        /* quit application */
    #endif
}

//static void draw_hello(void) {
//
//    VMWSTR s;                    /* string's buffer */
//    int x;                        /* string's x coordinate */
//    int y;                        /* string's y coordinate */
//    int wstr_len;                /* string's length */
//    vm_graphic_color color;        /* use to set screen and text color */
//
//
//    /* get string content from resource */
//    s = (VMWSTR)vm_res_get_string(STR_ID_HELLO);
//
//    /* calculate string length*/
//    wstr_len = vm_graphic_get_string_width(s);
//
//    /* calculate string's coordinate */
//    x = (vm_graphic_get_screen_width() - wstr_len) / 2;
//    y = (vm_graphic_get_screen_height() - vm_graphic_get_character_height()) / 2;
//
//    /* set screen color */
//    color.vm_color_565 = VM_COLOR_WHITE;
//    vm_graphic_setcolor(&color);
//
//    /* fill rect with screen color */
//    vm_graphic_fill_rect_ex(layer_hdl[0], 0, 0, vm_graphic_get_screen_width(), vm_graphic_get_screen_height());
//
//    /* set text color */
//    color.vm_color_565 = VM_COLOR_BLUE;
//    vm_graphic_setcolor(&color);
//
//    /* output text to layer */
//    vm_graphic_textout_to_layer(layer_hdl[0],x, y, s, wstr_len);
//
//    /* flush the screen with text data */
//    vm_graphic_flush_layer(layer_hdl, 1);
//}

/****************************************************************************
 *  profile content
 ****************************************************************************/
void app_client_callback_init(vm_bt_gatt_client_callback_t *gattc_cb)
{
    gattc_cb->register_client = app_client_register_client_callback;
    gattc_cb->scan_result = app_client_vmt_scan_result_callback ;
    gattc_cb->connection = app_client_connection_callback;
    gattc_cb->listen = app_client_listen_callback;
    gattc_cb->set_advertisement_data = app_client_set_adv_data_callback;
    gattc_cb->search_complete = app_client_search_complete_callback;
    gattc_cb->search_result = app_client_search_result_callback;
    gattc_cb->get_characteristic = app_client_get_characteristic_callback;
    gattc_cb->get_descriptor = app_client_get_descriptor_callback;
    gattc_cb->get_included_service = app_client_get_included_service_callback;
    gattc_cb->register_for_notification = app_client_register_for_notification_callback;
    gattc_cb->notify = app_client_notify_callback;
    gattc_cb->read_characteristic = app_client_read_characteristic_callback;
    gattc_cb->write_characteristic = app_client_write_characteristic_callback;
    gattc_cb->read_descriptor = app_client_read_descriptor_callback;
    gattc_cb->write_descriptor = app_client_write_descriptor_callback;
    gattc_cb->execute_write = app_client_execute_write_callback;
    gattc_cb->read_remote_rssi = app_client_read_remote_rssi_callback;
    gattc_cb->get_device_type = app_client_get_device_type_callback;
    return;
}

void app_client_check_bt_on_off(void)
{
    vm_bt_cm_init(app_client_bt_init_cb,0x00080000 | 0x00000001,NULL);
    if (VM_BT_CM_POWER_ON == vm_bt_cm_get_power_status())
    {
        app_client_init();
    }
    else
    {
        vm_bt_cm_switch_on();
    }

}
void app_client_bt_init_cb(VMUINT evt,void * param,void * user_data)
{
    if (0x00000001 == evt || 0x00080000 == evt)
    {
        app_client_init();
    }
}

void app_client_init(void)
{
    vm_log_debug("[AppClient] appc_init state %d!\n", g_appc_cntx.state);
    if(g_appc_cntx.state == APPC_STATUS_DISABLED)
    {
        g_appc_cntx.state = APPC_STATUS_ENABLING;
        memset(g_appc_cntx.uid,0x0,sizeof(g_appc_cntx.uid));
        memcpy(g_appc_cntx.uid, g_appc_uid, sizeof(g_appc_cntx.uid));
        app_client_callback_init(&g_appc_cb);
        vm_bt_gatt_client_register(g_appc_cntx.uid, &g_appc_cb);
        vm_log_debug("[AppClient] appc_init_end");
    }
    return;
}

void app_client_register_client_callback(void *context_handle, VMBOOL status, VMUINT8 app_uuid[16])
{
	if (exiting) return;
    vm_log_debug("[AppClient] reg_cb status %d state %d!\n", status, g_appc_cntx.state);
    if(memcmp(app_uuid, g_appc_cntx.uid, sizeof(g_appc_cntx.uid)) == 0)
    {
        if(g_appc_cntx.state == APPC_STATUS_ENABLING)
        {
            if(status == 0)
            {
                vm_log_debug("[AppClient] reg_cb status_ok %d state_ok %d!\n", status, g_appc_cntx.state);
                g_appc_cntx.context_handle = context_handle;
                g_appc_cntx.state = APPC_STATUS_ENABLED;
                //vm_bt_gatt_client_listen(context_handle, VM_TRUE);
                vm_bt_gatt_client_scan(context_handle, VM_TRUE);
            }
            else
            {
                g_appc_cntx.context_handle = NULL;
                g_appc_cntx.state = APPC_STATUS_DISABLED;
            }
        }
        else if(g_appc_cntx.state == APPC_STATUS_DISABLING)
        {
            if(status == 0)
            {
                g_appc_cntx.context_handle = NULL;
                g_appc_cntx.state = APPC_STATUS_DISABLED;
            }
            else
            {
                g_appc_cntx.state = APPC_STATUS_ENABLED;
            }
        }
    }
    vm_log_debug("[AppClient] reg_cb -!\n");
}

void app_client_listen_callback(void *context_handle, VMBOOL status)
{
	if (exiting) return;
    vm_log_debug("[AppClient] app_client_listen_callback status %d\n", status);
    if ((g_appc_cntx.context_handle == context_handle) && (NULL != context_handle))
    {
        if(status == 0)
        {
            vm_log_debug("[AppClient] app_client_listen_callback statusok %d!\n", status);
            vm_bt_gatt_client_scan(context_handle, VM_TRUE);
        }
    }
}

const int MAX_INDEX = 10;
int busy = 0;
int next_beacon = 0;

static VMUINT32 app_client_get_free_index(vm_bt_gatt_address_t *bd_addr)
{
    VMUINT32 idx;

    for (idx = 0; idx < MAX_INDEX; idx++)
    {
    	////  If already exists, return the same index.
        if (APP_CLIENT_BD_ADDR_LIST[idx].data[0] == bd_addr->data[0]
         && APP_CLIENT_BD_ADDR_LIST[idx].data[1] == bd_addr->data[1]
         && APP_CLIENT_BD_ADDR_LIST[idx].data[2] == bd_addr->data[2]
         && APP_CLIENT_BD_ADDR_LIST[idx].data[3] == bd_addr->data[3]
         && APP_CLIENT_BD_ADDR_LIST[idx].data[4] == bd_addr->data[4]
         && APP_CLIENT_BD_ADDR_LIST[idx].data[5] == bd_addr->data[5])
        {
            return idx;
        }
        ////  Else return the next empty index.
        if (APP_CLIENT_BD_ADDR_LIST[idx].data[0] == 0
         && APP_CLIENT_BD_ADDR_LIST[idx].data[1] == 0
         && APP_CLIENT_BD_ADDR_LIST[idx].data[2] == 0
         && APP_CLIENT_BD_ADDR_LIST[idx].data[3] == 0
         && APP_CLIENT_BD_ADDR_LIST[idx].data[4] == 0
         && APP_CLIENT_BD_ADDR_LIST[idx].data[5] == 0)
        {
            return idx;
        }
    }
    return 100;
}

void connect_next_beacon(void *context_handle) {
	//  Connect to next beacon in my list of discovered beacons, to get RSSI.
	if (exiting) return;
    VMUINT32 idx;
    for (idx = 0; idx < MAX_INDEX; idx++)
    {
    	int i = (idx + next_beacon) % MAX_INDEX;
        if (APP_CLIENT_BD_ADDR_LIST[i].data[0] != 0
         || APP_CLIENT_BD_ADDR_LIST[i].data[1] != 0
         || APP_CLIENT_BD_ADDR_LIST[i].data[2] != 0
         || APP_CLIENT_BD_ADDR_LIST[i].data[3] != 0
         || APP_CLIENT_BD_ADDR_LIST[i].data[4] != 0
         || APP_CLIENT_BD_ADDR_LIST[i].data[5] != 0)
        {
        	vm_bt_gatt_address_t *bd_addr = &(APP_CLIENT_BD_ADDR_LIST[i]);
        	next_beacon = (i + 1) % MAX_INDEX;
            vm_log_debug("*****connecting to [%x:%x:%x:%x:%x:%x]",
    			bd_addr->data[5],
    			bd_addr->data[4],
    			bd_addr->data[3],
    			bd_addr->data[2],
    			bd_addr->data[1],
    			bd_addr->data[0]
                );
        	busy = 1;
        	vm_bt_gatt_client_connect(context_handle, bd_addr, VM_TRUE);
        	return;
        }
    }
    vm_log_debug("no beacons to be connected");
}

void app_client_vmt_scan_result_callback(void *context_handle, vm_bt_gatt_address_t *bd_addr, VMINT32 rssi, VMUINT8 eir_len, VMUINT8 *eir)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    VMUINT32 idx;
    VMUINT8 server_addr[VM_BT_GATT_ADDRESS_SIZE] = {0xC9,0x92,0x65,0x46,0x5B,0x7E};
    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ////vm_log_debug("[AppClient] scan_cb bd_addr %x:%x:%x:%x:%x:%x",
        ////bd_addr->data[0],bd_addr->data[1],bd_addr->data[2],bd_addr->data[3],bd_addr->data[4],bd_addr->data[5]);

	if (exiting) return;
    ////  Process only our beacons.
    if (bd_addr->data[5] == 0xfd ||
		bd_addr->data[5] == 0xcf ||
		bd_addr->data[5] == 0xdc) {
    	//  Proceed to get RSSI.
        vm_log_debug("*****scanned our beacon [%x:%x:%x:%x:%x:%x]",
            bd_addr->data[5],
            bd_addr->data[4],
            bd_addr->data[3],
            bd_addr->data[2],
            bd_addr->data[1],
            bd_addr->data[0]
            );
    }
    else {
    	//  Skip the unknown beacon.
        vm_log_debug("skipped unknown beacon [%x:%x:%x:%x:%x:%x]",
			bd_addr->data[5],
			bd_addr->data[4],
			bd_addr->data[3],
			bd_addr->data[2],
			bd_addr->data[1],
			bd_addr->data[0]
            );
    	return;
    }
    ////  Stop adding new beacons if we have seen 10 of our beacons already.  TODO: Support more than 10 beacons.
    idx = app_client_get_free_index(bd_addr);
    if (idx >= 10) {
		vm_log_debug("[AppClient] *****exceeded dev index:%d", idx);
		return;
    }
    ////  Remember all my beacons that were scanned.
	vm_log_debug("[AppClient] allocated dev index:%d", idx);
	memcpy(&APP_CLIENT_BD_ADDR_LIST[idx], bd_addr, sizeof(vm_bt_gatt_address_t));
	////  Connect one beacon at a time. The board doesn't support multiple connections.
    if(g_appc_cntx.context_handle == context_handle && !busy)
    {
        ////  Continue scanning for my beacons but connect one at a time.
		////vm_bt_gatt_client_scan(context_handle,VM_FALSE);
    	connect_next_beacon(context_handle);
    }
}

static char buffer[64];

static void app_client_read_remote_rssi_callback(void *context_handle, VMBOOL status, vm_bt_gatt_address_t *bd_addr, VMINT32 rssi)
{
	if (exiting) return;
    if (rssi)
    {
        if (!status)
        {
            vm_log_debug("*****RSSI for [%x:%x:%x:%x:%x:%x] is [%d]",
                bd_addr->data[5],
                bd_addr->data[4],
                bd_addr->data[3],
                bd_addr->data[2],
                bd_addr->data[1],
                bd_addr->data[0],
                rssi
                );
            //  Send the MAC address and RSSI to SIGFOX e.g. "AT$SS=31323334\r".
            snprintf(buffer, sizeof(buffer), "AT$SS=%02X%02X%02X%02X%02X%02X%02X\r",
                bd_addr->data[5],
                bd_addr->data[4],
                bd_addr->data[3],
                bd_addr->data[2],
                bd_addr->data[1],
                bd_addr->data[0],
                rssi < 0 ? -rssi : rssi
                );
            send_uart_data((unsigned char *) buffer, strlen(buffer));
        }
        else
        {
            vm_log_debug("******RSSI failed for [%x:%x:%x:%x:%x:%x]",
                bd_addr->data[5],
                bd_addr->data[4],
                bd_addr->data[3],
                bd_addr->data[2],
                bd_addr->data[1],
                bd_addr->data[0]
                );
        }
    }
    ////  Disconnect after getting RSSI.  Continue with next beacon.
    vm_bt_gatt_client_disconnect(context_handle, bd_addr);
    g_appc_cntx.context_handle = context_handle;
    busy = 0;
    connect_next_beacon(context_handle);
}

int connect_failed_count = 0;

void app_client_connection_callback(vm_bt_gatt_connection_t *conn, VMBOOL connected, vm_bt_gatt_address_t *bd_addr)
{
	if (exiting) return;
    vm_log_debug("[AppClient]connection_cb connected[%d]", connected);
    if (connected)
    {
        VM_TIMER_ID_NON_PRECISE timer_id = 0;
        RSSI_T *rssi = (RSSI_T *)vm_calloc(sizeof(RSSI_T));
        rssi->conn.connection_handle = conn->connection_handle;
        rssi->conn.context_handle = conn->context_handle;

        ////
        if (!bd_addr) {
            vm_log_debug("skipped unaddressed beacon");
        	return;
        }
        //  Wait a while before getting the RSSI.
		memcpy(rssi->bd_addr.data, bd_addr->data, sizeof(rssi->bd_addr.data));
        timer_id = vm_timer_create_non_precise(2000, (vm_timer_non_precise_callback)timer_cb, rssi);
        ////vm_bt_gatt_client_read_remote_rssi(conn->context_handle, bd_addr); ////
    }
    else {
        ////  If not connected, we disconnect and continue with next beacon.
        vm_bt_gatt_client_disconnect(conn->context_handle, bd_addr);
        g_appc_cntx.context_handle = conn->context_handle;
        busy = 0;

        ////  Restart if too many errors.
        connect_failed_count++;
        if (connect_failed_count == 2) {
            VM_TIMER_ID_NON_PRECISE timer_id = 0;
        	exiting = 1;
        	connect_failed_count = 0;
            vm_log_debug("*****restarting in 5 seconds");
            timer_id = vm_timer_create_non_precise(5000, (vm_timer_non_precise_callback)timer_cb, 0x2);
        	return;
        	//app_client_deinit(); app_client_init();
        }
        connect_next_beacon(conn->context_handle);
        return;
    }

    ////return;
    ////  Read more beacons.

    if((memcmp(bd_addr->data, appc_conn_cntx->bdaddr, VM_BT_GATT_ADDRESS_SIZE) == 0) && (NULL != bd_addr))
    {
        vm_log_debug("[AppClient] find in list!\n");
        appc_conn_cntx->connection_handle = conn->connection_handle;
        if(connected  && (appc_conn_cntx->conn_status != APPC_STATUS_CONNECTED))
        {
            //Do next step to discover all
            vm_log_debug("[AppClient] have connected,find in list!\n");
            //vm_bt_gatt_client_listen(conn->connection_handle, FALSE);
            appc_conn_cntx->conn_status = APPC_STATUS_CONNECTING;
            //vm_bt_gatt_client_search_service(conn, NULL);
        }
        else if(!connected)
        {
            //vm_bt_gatt_client_listen(conn->connection_handle, TRUE);
            //vm_free(appc_conn_cntx);
            vm_log_debug("[AppClient] connection faield");
        }
        return;
    }
    if(connected)
    {
        vm_log_debug("[AppClient] connect success, and not find in list!\n");
        if (appc_conn_cntx == NULL)
        {
            appc_conn_cntx = (AppClientConnCntx *)vm_malloc(sizeof(AppClientConnCntx));
        }
        memset(appc_conn_cntx, 0x0, sizeof(AppClientConnCntx));
        memcpy(appc_conn_cntx->bdaddr, bd_addr->data, VM_BT_GATT_ADDRESS_SIZE);
        appc_conn_cntx->connection_handle = conn->connection_handle;
        //vm_bt_gatt_client_listen(conn->context_handle, FALSE);
        appc_conn_cntx->conn_status = APPC_STATUS_CONNECTING;
        //vm_bt_gatt_client_search_service(conn, NULL);
    }
}

void app_client_search_complete_callback(void *context_handle, VMBOOL status)
{
	if (exiting) return;
    vm_log_debug("[AppClient] app_client_search_complete_callback status %d!\n", status);
    if(g_appc_cntx.context_handle == context_handle)
    {
        vm_log_debug("[AppClient] seach complete !\n");
        if(status == 0)
        {
            if(appc_conn_cntx->conn_status == APPC_STATUS_CONNECTING)
            {
                appc_conn_cntx->conn_status = APPC_STATUS_CONNECTED;
            }
        }
        return;
    }

}
/*
VMUINT16 app_convert_array_to_uuid16(vm_bt_uuid_with_length_t *uu)
{
    VMUINT16 uuid = 0;

    if(uu->length == 2)
    {
        uuid = ((VMUINT16)uu->uuid[1]) << 8 | uu->uuid[0];
    }
    else if(uu->length == 16)
    {
        uuid = ((VMUINT16)uu->uuid[13]) << 8 | uu->uuid[12];
    }

    return uuid;
}


VMINT  get_appc_service_type(VM_ATT_UUID uu)
{
    VMUINT16 uuid = 0;
    VMINT type = -1;

    uuid = app_convert_array_to_uuid16(uu);
    switch(uuid)
    {
    case IMMEDIATE_ALERT_SERVICE:
        type = IAS_SERVICE_TYPE;
        break;
    default:
        break;
    }
    vm_log_debug("[AppClient] get_appc_service_type %d!\n", type);
    return type;
}
*/

void app_client_search_result_callback(vm_bt_gatt_connection_t *conn, vm_bt_gatt_service_info_t *uuid)
{
	if (exiting) return;
    //VMINT type = -1;

    vm_log_debug("[AppClient] app_client_search_result_callback !\n");
    //type = get_appc_service_type(uuid->uuid);
    //if(type == -1)
    //    return;
    //else if (type == 0)
    if(memcmp(&(uuid->uuid.uuid.uuid),&appc_srv_uuid, (sizeof(VMUINT8) * 16)) == 0)
    {
        //vm_log_debug("[AppClient] find in list !\n");
        vm_log_debug("[AppClient]service2 has been find !\n");
        if(appc_conn_cntx->connection_handle == conn->connection_handle)
        {
            //appc_srv_uuid = (vm_bt_gatt_service_info_t *)vm_malloc(sizeof(vm_bt_gatt_service_info_t));
            //memset(appc_srv_uuid, 0x0, sizeof(vm_bt_gatt_service_info_t));
            //appc_srv_uuid = uuid;
            vm_log_debug("[AppClient] to get character !\n");
            vm_bt_gatt_client_get_characteristic(conn, uuid, NULL);
            return;
        }
    }
}


void app_client_get_characteristic_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                    vm_bt_gatt_client_characteristic_t *ch, VM_BT_GATT_CHAR_PROPERTIES properties)
{
	if (exiting) return;
    //VMUINT16               char_uuid= appc_convert_array_to_uuid16(*ch->ch_uuid);
    //VMUINT16               svc_uuid = appc_convert_array_to_uuid16(ch->svc_uuid->uuid);

    vm_log_debug("[AppClient] app_client_get_characteristic_callback status %d!\n", status);
    if(appc_conn_cntx->connection_handle == conn->connection_handle)
    {
        if(memcmp(&(ch->svc_uuid->uuid.uuid.uuid),&appc_srv_uuid, (sizeof(VMUINT8) * 16)) == 0)
        {
            if(memcmp(&(ch->ch_uuid->uuid.uuid),&appc_char_uuid, (sizeof(VMUINT8) * 16)) == 0)
            {
                vm_log_debug("[AppClient]service2 has been find !\n");
                if(status == 0)
                {
                    if (properties & (VM_BT_GATT_CHAR_PROPERTY_READ))
                    {
                        vm_log_debug("[AppClient] read service2  !\n");
                        vm_bt_gatt_client_read_characteristic(conn,ch,VM_BT_GATT_CLIENT_AUTH_REQ_NONE);
                    }
                    else if (properties & (VM_BT_GATT_CHAR_PROPERTY_WRITE || VM_BT_GATT_CHAR_PROPERTY_WRITE_WO_RESPONSE))
                    {
                        char_value.data[0] = 0;
                        char_value.length = 1;
                        vm_log_debug("[AppClient] write service2 !\n");
                        vm_bt_gatt_client_write_characteristic(conn,
                                                       ch,
                                                       &char_value,
                                                       VM_BT_GATT_CLIENT_WRITE_TYPE_PREPARE,
                                                       VM_BT_GATT_CLIENT_AUTH_REQ_NONE);
                    }
                }
            }

        }
        #if 0
        if((char_uuid == ALERT_LEVEL_CHAR_UUID)
            && (svc_uuid == IMMEDIATE_ALERT_SERVICE))
        {
            vm_log_debug("[AppClient] is immediately service find !\n");
            if(status == 0)
            {
                if (properties == (VM_BT_GATT_CHAR_PROPERTY_WRITE || VM_BT_GATT_CHAR_PROPERTY_WRITE_WO_RESPONSE))
                {
                    char_value.value[0] = 0;
                    char_value.len = 1;
                    vm_log_debug("[AppClient] immediately service write !\n");
                    vm_bt_gatt_client_write_characteristic(conn,
                                                   ch,
                                                   &char_value,
                                                   VM_BT_GATTC_WRITE_TYPE_PREPARE,
                                                   VM_BT_GATTC_AUTH_REQ_NONE);
                }
                else if (properties == VM_BT_GATT_CHAR_PROPERTY_READ)
                {
                    vm_log_debug("[AppClient] immediately service read !\n");
                    vm_bt_gatt_client_read_characteristic(conn,ch,VM_BT_GATTC_AUTH_REQ_NONE);
                }
            }
        }
        #endif
        else
        {
            if(status == 0)
            {
                vm_bt_gatt_client_get_characteristic(conn, ch->svc_uuid, ch->ch_uuid);
            }
        }
    }
    return ;
}

void app_client_write_characteristic_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_characteristic_t *ch)
{
	if (exiting) return;
    //VMINT  type = -1;
    //VMUINT16  uuid = 0;

    vm_log_debug("[AppClient] app_client_write_characteristic_callback status %d!\n", status);
    if(appc_conn_cntx->connection_handle == conn->connection_handle)
    {
    #if 0
        type = get_appc_service_type(ch->svc_uuid->uuid);
        if(type == IAS_SERVICE_TYPE)
        {
            uuid = appc_convert_array_to_uuid16(*ch->ch_uuid);
            if(uuid == ALERT_LEVEL_CHAR_UUID)
            {
                if(status == 0)
                {
                    //todo
                    vm_log_debug("[AppClient] is immediately service write done!\n");
                }
            }
        }
    #endif
        if(memcmp(&(ch->svc_uuid->uuid.uuid.uuid),&appc_srv_uuid, (sizeof(VMUINT8) * 16)) == 0)
        {
            if(memcmp(&(ch->ch_uuid->uuid.uuid),&appc_char_uuid, (sizeof(VMUINT8) * 16)) == 0)
            {
                if(status == 0)
                {
                    /*you can set exeute as 0,1,2*/
                    vm_bt_gatt_client_execute_write(conn,1);
                    vm_log_debug("[AppClient] service execute write execute=1 !\n");
                }
            }
        }

    }
    return;
}

void app_client_execute_write_callback(vm_bt_gatt_connection_t *conn, VMBOOL status)
{
	if (exiting) return;
    vm_log_debug("[AppClient] app_client_execute_write_callback !\n");

    if(appc_conn_cntx->connection_handle == conn->connection_handle)
    {
        if(status == 0)
        {
            vm_log_debug("[AppClient] service execute write success !\n");
        }
    }
    return;
}

/* Reports result of a GATT read operation */
void app_client_read_characteristic_callback(vm_bt_gatt_connection_t *conn, VMBOOL status,
                                vm_bt_gatt_client_characteristic_t *ch, vm_bt_gatt_attribute_value_t *value)
{
	if (exiting) return;
    //VMINT  type = -1;
    //VMUINT16  uuid = 0;

    vm_log_debug("[AppClient] app_client_read_characteristic_callback status %d!\n", status);
    if(appc_conn_cntx->connection_handle == conn->connection_handle)
    {
    #if 0
        type = get_appc_service_type(ch->svc_uuid->uuid);
        if(type == IAS_SERVICE_TYPE)
        {
            uuid = appc_convert_array_to_uuid16(*ch->ch_uuid);
            if(uuid == ALERT_LEVEL_CHAR_UUID)
            {
                if(status == 0)
                {
                    //todo
                    vm_log_debug("[AppClient] is immediately service read done value %x:%x:%x:%x:%x:%x!\n", value->value[0],value->value[0],
                                 value->value[2],value->value[3],value->value[4],value->value[5]);
                }
            }
        }
    #endif
        if(memcmp(&(ch->svc_uuid->uuid.uuid.uuid),&appc_srv_uuid, (sizeof(VMUINT8) * 16)) == 0)
        {
            if(memcmp(&(ch->ch_uuid->uuid.uuid),&appc_char_uuid, (sizeof(VMUINT8) * 16)) == 0)
            {
                if(status == 0)
                {
                    vm_log_debug("[AppClient] service read done value %x:%x:%x:%x:%x:%x!\n", value->data[0],value->data[0],
                                 value->data[2],value->data[3],value->data[4],value->data[5]);
                    /*to test write API*/
                    {
                        char_value.data[0] = 0;
                        char_value.length = 1;
                        vm_log_debug("[AppClient] write service2 !\n");
                        vm_bt_gatt_client_write_characteristic(conn,
                                                       ch,
                                                       &char_value,
                                                       VM_BT_GATT_CLIENT_WRITE_TYPE_PREPARE,
                                                       VM_BT_GATT_CLIENT_AUTH_REQ_NONE);
                    }
                }
            }
        }
    }
    return;
}

VMINT app_client_deinit(void)
{
    vm_log_debug("[AppClient] app_client_deinit state %d!\n", g_appc_cntx.state);
    if((g_appc_cntx.state == APPC_STATUS_DISABLED)
        || (g_appc_cntx.state == APPC_STATUS_DISABLING))
        return 0;

    if(g_appc_cntx.state == APPC_STATUS_ENABLED)
    {
        {
            client_gatt_conn.context_handle = g_appc_cntx.context_handle;
            client_gatt_conn.connection_handle = appc_conn_cntx->connection_handle;
            if((appc_conn_cntx->conn_status == APPC_STATUS_CONNECTED)
                || (appc_conn_cntx->conn_status == APPC_STATUS_CONNECTING))
            {
                client_bd_addr = (vm_bt_gatt_address_t *)vm_malloc(sizeof(vm_bt_gatt_address_t));
                memset(client_bd_addr, 0x0, sizeof(vm_bt_gatt_address_t));
                memcpy(client_bd_addr->data, appc_conn_cntx->bdaddr, VM_BT_GATT_ADDRESS_SIZE);
                vm_bt_gatt_client_disconnect(&client_gatt_conn, client_bd_addr);
            }
        }
    }

    vm_bt_gatt_client_deregister(g_appc_cntx.context_handle);
    g_appc_cntx.state = APPC_STATUS_DISABLED;
    //vm_free(appc_srv_uuid);

    if (appc_conn_cntx)
    {
        vm_free(appc_conn_cntx);
    }
    if (client_bd_addr)
    {
        vm_free(client_bd_addr);
    }
    return 1;
}
