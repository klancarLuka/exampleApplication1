/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 	/* included headers */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

	/* button definition */
	
#define SW1_NODE DT_ALIAS(sw0) /* sw0 is actually button SW1 on nucleo_wb55rg */

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});

static struct gpio_callback button_cb_data; /* callback data definiton */

	/* led gpio & start state definition */
#define LED DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED, gpios);
static uint8_t led_state = 0;


	/* bt uuid */
static struct bt_uuid_128 led_state_char_notify_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x9c85a727, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120002));
static struct bt_uuid_128 led_state_char_read_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x9c85a728, 0xb7f1, 0x11ec, 0xb909, 0x0242ac120002));

#define LED_SERVICE_UUID_VAL \
  BT_UUID_128_ENCODE(0xf7547938, 0x68ba, 0x11ec, 0x90d6, 0x0242ac120003)

static struct bt_uuid_128 led_svc_uuid = 
	BT_UUID_INIT_128(LED_SERVICE_UUID_VAL);
	
	/* advertisement data */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, LED_SERVICE_UUID_VAL),
};

/* read state function */
static ssize_t read_led_state(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {

  char izhod[11] = "LED is OFF"; //size("LED is OFF" + null char) = 11 

  if (led_state == 0){
	izhod[8] = 'F';
	izhod[9] = 'F';
  } else {
 	izhod[9] = ' ';
	izhod[8] = 'N';
  }

  return bt_gatt_attr_read(conn, attr, buf, len, offset, izhod, sizeof(izhod));
}

/* change indication function */
static uint8_t simulate_vnd;

static void led_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

/* bt service definition */
BT_GATT_SERVICE_DEFINE(led_svc, BT_GATT_PRIMARY_SERVICE(&led_svc_uuid),
	BT_GATT_CHARACTERISTIC(&led_state_char_notify_uuid.uuid,	/*notify chrc*/
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
	BT_GATT_CCC(led_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&led_state_char_read_uuid.uuid,		/* read chrc*/
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_led_state, NULL, &led_state));
  
 /* calback functions */           
 static void connected (struct bt_conn *conn, uint8_t err){
 	if(err){
 		printk("Connection failed\n");
 	} else {
 		printk("Connected\n");
 	}
 }
 static void disconnected (struct bt_conn *conn, uint8_t reason){
	printk("Disconnected\n");
 }
 
 static struct bt_conn_cb conn_callbacks = {
 	.connected = connected,
 	.disconnected = disconnected,
};
              
/* notification function */
int bt_notify(){

	char izhod[11] = "LED is OFF"; //size("LED is OFF" + null char) = 11 

	if (led_state == 0){
		izhod[8] = 'F';
		izhod[9] = 'F';
	} else {
		izhod[9] = ' ';
		izhod[8] = 'N';
	}
	int rc;
	rc = bt_gatt_notify(NULL, &led_svc.attrs[1], izhod, sizeof(izhod));
	
	return rc == -ENOTCONN ? 0 : rc;
}
      
/* on interupt function */
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	gpio_pin_toggle_dt(&led);
	led_state = (led_state + 1) % 2;
	
	if (bt_notify() != 0){
		printk("Notification error\n");
	}
}
                     
void main(void)
{
	/* led config + tests*/
	if(!device_is_ready(led.port)){
		printk("Error: Led is not ready\n");
	}
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, led_state);

	/* button config + tests */
	int ret;
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}
	
	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}
	
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	
	
	/* Bluetooth config + tests */
	int err;

	// initialize BLE
	err = bt_enable(NULL);
	if (err) {
	  printk("Bluetooth init failed (err %d)\n", err);
	  return;
	}
	printk("Bluetooth initialized\n");

	// BT_GAP_ADV_FAST_INT_MIN_2 -> 100ms
	// BT_GAP_ADV_FAST_INT_MAX_2 -> 150ms
	err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL), ad, ARRAY_SIZE(ad), NULL, 0);
	
	if (err) {
	  printk("Advertising failed to start (err %d)\n", err);
	  return;
	}
	printk("Advertising successfully started\n");
	
	/* callback registration */
	bt_conn_cb_register(&conn_callbacks);
	
	if (led.port) {
		while (1) {
			/* empty main loop */
			/* additionaly, we could add a delay 
			for lower power consumption */
		}
		
	}
}









