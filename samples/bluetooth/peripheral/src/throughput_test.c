#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <settings/settings.h>


static uint8_t gatt_svr_throughput_static_val[512];
static uint16_t gatt_throuput_svr_handle = 0xffff;
static uint8_t ccd = 0;
static uint8_t allow_tx = 0;
static int
gatt_svr_chr_access_throughput(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

void throughput_run(uint16_t conn_handle);


static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service: Security test. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x00ff),
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /*** Characteristic: Random number generator. */
            .uuid = BLE_UUID16_DECLARE(0xff01),
            .access_cb = gatt_svr_chr_access_throughput,
            .flags = BLE_GATT_CHR_F_NOTIFY|BLE_GATT_CHR_F_READ,
        }, {
            /*** Characteristic: Static value. */
            .uuid =BLE_UUID16_DECLARE(0xff02),
            .access_cb = gatt_svr_chr_access_throughput,
            .flags = BLE_GATT_CHR_F_WRITE_NO_RSP
        }, {
            0, /* No more characteristics in this service. */
        } },
    },

    {
        0, /* No more services. */
    },
};



static int
gatt_svr_chr_access_throughput(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    const ble_uuid_t *uuid;
    int rand_num;
    int rc = 0;

    uuid = ctxt->chr->uuid;

    /* Determine which characteristic is being accessed by examining its
     * 128-bit UUID.
     */
	MODLOG_DFLT(DEBUG, "access callback called\n");
	

    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(0xff01)) == 0) {

        /* Respond with a 32-bit random number. */
        rand_num = rand();
        rc = os_mbuf_append(ctxt->om, &rand_num, sizeof rand_num);
		MODLOG_DFLT(DEBUG, "uuid 0xff02 called\n");
		ccd = 1;
		allow_tx = 1;
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(0xff03)) == 0) {
		MODLOG_DFLT(DEBUG, "uuid 0xff03 called\n");
        switch (ctxt->op) {
        case BLE_GATT_CHR_F_WRITE_NO_RSP:
            return rc == 0;
        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }else{
		ccd = 1;
		allow_tx = 1;
		return 0;
    }

    /* Unknown characteristic; the nimble stack should not have called this
     * function.
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}


int
gatt_svr_chr_notify_throughput(uint16_t conn_handle)
{
	int rc = -1;

	if((conn_handle != 0xffff) && (gatt_throuput_svr_handle!= 0xffff) && (ccd == 1) && (allow_tx)){
	
		struct os_mbuf *om;
		allow_tx = 0;

		om = ble_hs_mbuf_from_flat(gatt_svr_throughput_static_val, sizeof gatt_svr_throughput_static_val);

		rc = ble_gattc_notify_custom(conn_handle, gatt_throuput_svr_handle, om);
		MODLOG_DFLT(DEBUG, "notify send status %d\n",rc);
	}
	return rc;
}
void throughput_ccd_set(uint8_t value){

	ccd = value;

}

void throughput_allow_tx(uint8_t allow){
	allow_tx = allow;


}


void throughput_set_handle(uint16_t svr_handle){

	gatt_throuput_svr_handle = svr_handle;
	MODLOG_DFLT(DEBUG, "notify handle set to %d\n",gatt_throuput_svr_handle);


}


void throughput_run(uint16_t conn_handle){

	gatt_svr_chr_notify_throughput(conn_handle);


}

//Debug pins P1.01 ~ P1.08
static const int buttons[8] = {0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28};

static int pin_values[8];
static void througput_pins_init(void ){
	int j = 0;
	for(;j<8;j++){
		hal_gpio_init_out(buttons[j], 0);
		pin_values[j]=0;
	}
}
extern void throughput_pins_set(int index, int value);
void throughput_pins_set(int index, int value){
	hal_gpio_write(buttons[index],value);
	pin_values[index]=value;

}
extern void throughput_pins_toggle(int index);
void throughput_pins_toggle(int index){
	pin_values[index] = !pin_values[index];
	hal_gpio_write(buttons[index],pin_values[index]);

}
							 

int throughput_test_init(void ){
	int rc;
	uint16_t cnt=0;
	for(;cnt<sizeof gatt_svr_throughput_static_val;cnt++){
		gatt_svr_throughput_static_val[cnt] = cnt&0x00ff;
	}

	rc = ble_gatts_count_cfg(gatt_svr_svcs);
	if (rc != 0) {
		return rc;
	}

	rc = ble_gatts_add_svcs(gatt_svr_svcs);
	througput_pins_init();
	return rc;
	
	
}


