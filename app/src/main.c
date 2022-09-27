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
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

	/* button definition */
	
#define SW1_NODE DT_ALIAS(sw0) /* sw0 is actually button SW1 on nucleo_wb55rg */

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

	/* led gpio & start state definition */
#define LED DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED, gpios);
static uint8_t led_state = false;


void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

                           
void main(void)
{
	/* led config + tests*/
	if(!device_is_ready(led.port)){
		printk("Error: Led %s is not ready\n",
		       led.port);
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


	if (led.port) {
		bool previouslyClicked = false;
		while (1) {
			int val = gpio_pin_get_dt(&button);
			if (val >= 0) {
				if (val == 0) {
					if (previouslyClicked) { 
					/*button was pressed and released */
						previouslyClicked = false;			
						ret = gpio_pin_toggle_dt(&led);
						led_state = gpio_pin_get_dt(&led);
						if (ret < 0) {
							return;
						}
					}					
				} else {
					if(!previouslyClicked){
						previouslyClicked = true;
					}
				}
			}
			k_msleep(1);
		}
		
	}
}









