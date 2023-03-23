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
#define SLEEP_TIME_MS   	(50)
#define _debounceTicks 		(50)	/* milisec that input state must be longer than this value to be valid  */
#define _clickTicks			(400)	/* milisec maximum time gap between each click  */
#define _pressTicks			(600)	/* milisec minimun hold time  */
#define _maxClicks 			(3)  	/* max number (1, 2, multi=3) of clicks of interest by registration of event functions */

#define MODULE_NAME			main_module
#define MODULE_LOG_LEVEL	LOG_LEVEL_INF
LOG_MODULE_REGISTER(MODULE_NAME, MODULE_LOG_LEVEL);

typedef enum {
    OCS_INIT = 0,
    OCS_DOWN = 1,
    OCS_UP = 2,
    OCS_COUNT = 3,
    OCS_PRESS = 6,
    OCS_PRESSEND = 7,
    UNKNOWN = 99
}button_state_t;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static struct gpio_callback pin_cb_data; /* Callback struct required by kernel */
static button_state_t _state;
static button_state_t _lastState;
unsigned long _startTime; // start of current input change to checking debouncing
int _nClicks;             // count the number of clicks with this variable
/******************************************************************************
* Function Prototypes
*******************************************************************************/
int led_init(struct gpio_dt_spec* p_led_device);
int button_init(struct gpio_dt_spec* p_button_device);
int button_int_init(struct gpio_dt_spec* p_button_device);
void button_handler(bool activeLevel);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins); 
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
	// button_int_init(&custom_button);
	button_init(&custom_button);
	while (1) {
		gpio_pin_toggle_dt(&custom_led);
		button_handler(gpio_pin_get_dt(&custom_button));
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

/**
 * @brief  Actually button handler that will be called by ISR 
 * @param dev 
 * @param cb 
 * @param pins 
 */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) /* Actually handler being called by ISR */
{
	LOG_INF("Button interrupt executed");
}

void _longPressStartFunc()
{
	LOG_INF("_longPressStartFunc() Callback ");
};
void _clickFunc()
{
	LOG_INF("_clickFunc() Callback ");
};
void _doubleClickFunc()
{
	LOG_INF("_doubleClickFunc() Callback ");
};
void _multiClickFunc()
{
	LOG_INF("_multiClickFunc() Callback ");
};
void _duringLongPressFunc()
{
	LOG_INF("_duringLongPressFunc() Callback ");
};
void _longPressStopFunc()
{
	LOG_INF("_longPressStopFunc() Callback ");
};


static void _newState(button_state_t nextState)
{
  _lastState = _state;
  _state = nextState;
} // _newState()

static void reset(void)
{
  _state = OCS_INIT;
  _lastState = OCS_INIT;
  _nClicks = 0;
  _startTime = 0;
}

//TODO: References: http://www.mathertel.de/Arduino/OneButtonLibrary.aspx
void button_handler(bool activeLevel)
{
	unsigned long now = k_uptime_get_32(); // current (relative) time in msecs.
	unsigned long waitTime = (now - _startTime);

	// Implementation of the state machine
	switch (_state)
	{
		case OCS_INIT:
		// waiting for level to become active.
		if (activeLevel)
		{
			_newState(OCS_DOWN);
			_startTime = now; // remember starting time
			_nClicks = 0;
		} // if
		break;

		case OCS_DOWN:
		// waiting for level to become inactive.

		if ((!activeLevel) && (waitTime < _debounceTicks))
		{
			// button was released to quickly so I assume some bouncing.
			_newState(_lastState);
		}
		else if (!activeLevel)
		{
			_newState(OCS_UP);
			_startTime = now; // remember starting time
		}
		else if ((activeLevel) && (waitTime > _pressTicks))
		{
			if (_longPressStartFunc)
				_longPressStartFunc();
			_newState(OCS_PRESS);
		} // if
		break;

		case OCS_UP:
		// level is inactive

		if ((activeLevel) && (waitTime < _debounceTicks))
		{
			// button was pressed to quickly so I assume some bouncing.
			_newState(_lastState); // go back
		}
		else if (waitTime >= _debounceTicks)
		{
			// count as a short button down
			_nClicks++;
			_newState(OCS_COUNT);
		} // if
		break;

		case OCS_COUNT:
		// dobounce time is over, count clicks

		if (activeLevel)
		{
			// button is down again
			_newState(OCS_DOWN);
			_startTime = now; // remember starting time
		}
		else if ((waitTime > _clickTicks) || (_nClicks == _maxClicks))
		{
			// now we know how many clicks have been made.

			if (_nClicks == 1)
			{
				// this was 1 click only.
				if (_clickFunc)
					_clickFunc();
			}
			else if (_nClicks == 2)
			{
				// this was a 2 click sequence.
				if (_doubleClickFunc)
					_doubleClickFunc();
			}
			else
			{
				// this was a multi click sequence.
				if (_multiClickFunc)
					_multiClickFunc();
			} // if

			reset();
		} // if
		break;

		case OCS_PRESS:
		// waiting for menu pin being release after long press.

		if (!activeLevel)
		{
			_newState(OCS_PRESSEND);
			_startTime = now;
		}
		else
		{
			// still the button is pressed
			if (_duringLongPressFunc)
				_duringLongPressFunc();
		} // if
		break;

		case OCS_PRESSEND:
		// button was released.

		if ((activeLevel) && (waitTime < _debounceTicks))
		{
			// button was released to quickly so I assume some bouncing.
			_newState(_lastState); // go back
		}
		else if (waitTime >= _debounceTicks)
		{
			if (_longPressStopFunc)
				_longPressStopFunc();
			reset();
		}
		break;

		default:
			// unknown state detected -> reset state machine
			_newState(OCS_INIT);
			break;
		} // if
}