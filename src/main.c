/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * Note: 
 * Tested on nRF Connect SDK Version : 2.0
 */

/******************************************************************************
* Includes
*******************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define SLEEP_TIME_MS   100

#define MODULE_NAME			main_module
#define MODULE_LOG_LEVEL	LOG_LEVEL_DBG
LOG_MODULE_REGISTER(MODULE_NAME, MODULE_LOG_LEVEL);

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static struct gpio_callback pin_cb_data; /* Callback struct required by kernel */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) /* Actually handler being called by ISR */
{
	LOG_INF("Button interrupt executed");
}


/******************************************************************************
* Function Prototypes
*******************************************************************************/
int led_init(struct gpio_dt_spec* p_led_device);
int button_init(struct gpio_dt_spec* p_button_device);
int button_int_init(struct gpio_dt_spec* p_button_device);

/******************************************************************************
* Function Definitions
*******************************************************************************/
void main(void)
{
	int ret;

	/* LED Init */
	static struct gpio_dt_spec custom_led = GPIO_DT_SPEC_GET(DT_NODELABEL(custom_led), gpios);
	led_init(&custom_led);

	/* Button Init */
	static struct gpio_dt_spec custom_button = GPIO_DT_SPEC_GET(DT_NODELABEL(custom_button0), gpios);
	button_int_init(&custom_button);

	while (1) {
		gpio_pin_toggle_dt(&custom_led);
        k_msleep(SLEEP_TIME_MS); 
	}
}

/**
 * @brief Init led from devicetree
 * @param p_led_device: pointer to led device struct 
 * @return 0 if success, error code otherwise int 
 */
int led_init(struct gpio_dt_spec* p_led_device)
{
	int ret;
	if (!device_is_ready(p_led_device->port)) {
		LOG_ERR("LED device not ready");
		return EIO;
	}

	ret = gpio_pin_configure_dt(p_led_device, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to config LED with err: %d", ret);
		return ret;
	}
	LOG_INF("LED init succesfully");
	return 0;
}

/**
 * @brief Init button from devicetree
 * @param p_button_device: pointer to button device struct 
 * @return 0 if success, error code otherwise int 
 */
int button_init(struct gpio_dt_spec* p_button_device)
{
	int ret = 0;
	if (!device_is_ready(p_button_device->port)) {
		LOG_ERR("Button device not ready");
		return EIO;
	}

	ret = gpio_pin_configure_dt(p_button_device, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to config button with err: %d", ret);
		return ret;
	}
	LOG_INF("BUTTON init succesfully");
	return ret;
}

int button_int_init(struct gpio_dt_spec* p_button_device)
{
	int ret;

	ret = button_init(p_button_device);
	if (ret < 0) {
		LOG_ERR("Failed to init button with err: %d", ret);
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(p_button_device, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to config interrupt with err: %d", ret);
		return ret;
	}

	gpio_init_callback(&pin_cb_data, button_pressed, BIT(p_button_device->pin));
	ret = gpio_add_callback(p_button_device->port, &pin_cb_data); /* Enable interrupt */
	if (ret < 0) {
		LOG_ERR("Failed to enable button interrupt with err: %d", ret);
		return ret;
	}
	return 0;
}
