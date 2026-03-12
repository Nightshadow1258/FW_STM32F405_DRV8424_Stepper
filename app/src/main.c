/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate AS5047P magnetic encoder driver.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/pwm.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/spi.h>

#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);


#define SLEEP_TIME_MS 1000
#define GPIO_LED_THREAD_STACK_SIZE 1024
#define GPIO_LED_THREAD_PRIORITY 3
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void gpio_led_thread_entry(void *p1, void *p2, void *p3)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	    /* Thread sleeps, yielding to high-prio sensor tasks */
    
    }
}
/* Define the low-priority thread */
K_THREAD_DEFINE(gpio_led_tid, GPIO_LED_THREAD_STACK_SIZE,
                gpio_led_thread_entry, NULL, NULL, NULL,
                GPIO_LED_THREAD_PRIORITY, 0, 0);

int main(void)
{
    

    printk("System Ready! LED should be blinking\n");

    while (1)
    {
        printk("Main While loop is running\n");
        k_msleep(2000);
    }

    return 0;
}
