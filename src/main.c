/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * Note: 
 * Tested on nRF Connect SDK Version : 2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>

/* STEP 7 - Change the sleep time from 1000 ms to 100 ms */
#define SLEEP_TIME_MS   1000

/* STEP 3.1 - Get the node identifier for button 1 through its alias sw0 */
#define SW1			DT_ALIAS(sw1)

/* STEP 3.2 - Get the device pointer. pin number, and pin's configuration flags through gpio_dt_spec */
static const struct gpio_dt_spec sw1_dev = GPIO_DT_SPEC_GET(SW1, gpios);

/* LED0_NODE is the devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


void main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}
	/* STEP 4 - Verify that the device is ready for use */
	if (!device_is_ready(sw1_dev.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	/* STEP 5 - Configure the pin connected to the button to be an input pin and set its hardware specifications */
	ret = gpio_pin_configure_dt(&sw1_dev, GPIO_INPUT);
	if (ret < 0) {
		return;
	}
	gpio_pin_set_dt(&led, 0);
	gpio_pin_set_dt(&led, 1);


	while (1) {
		/* STEP 6.1 - Read the status of the button and store it */
		bool val = gpio_pin_get_dt(&sw1_dev);
		/* STEP 6.2 - Update the LED to the status of the button */
		if(val)
		{
			gpio_pin_set_dt(&led, 1);
		}
		else 
		{
			gpio_pin_set_dt(&led, 0);
		}
		// k_msleep(SLEEP_TIME_MS); // Put the main thread to sleep for 100ms for power optimization
	}
}