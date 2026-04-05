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

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/dac.h>


LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

// LED Stuff
#define SLEEP_TIME_MS 1000
#define GPIO_LED_THREAD_STACK_SIZE 1024
#define GPIO_LED_THREAD_PRIORITY 3
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Motor control stuff
static const struct gpio_dt_spec enable_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_enable), gpios);
static const struct gpio_dt_spec dir_pin    = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_dir), gpios);
static const struct gpio_dt_spec mode1_pin  = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_mode1), gpios);
static const struct gpio_dt_spec mode0_pin  = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_mode0), gpios);

static const struct pwm_dt_spec step_pwm    = PWM_DT_SPEC_GET(DT_NODELABEL(drv_step));


void init_motor_driver(void)
{
    /* Configure GPIOs as output */
    gpio_pin_configure_dt(&enable_pin, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&mode1_pin, GPIO_OUTPUT_ACTIVE); // M1 High
    gpio_pin_configure_dt(&mode0_pin, GPIO_OUTPUT_ACTIVE); // M0 High

    /* Both M0 and M1 are now High, setting 1/16 microstepping */
}

void set_motor_speed(uint32_t frequency_hz)
{
    if (frequency_hz == 0) {
        pwm_set_pulse_dt(&step_pwm, 0);
        return;
    }

    uint32_t period_ns = 1000000000ULL / frequency_hz;
    uint32_t pulse_ns = period_ns / 2;

    pwm_set_dt(&step_pwm, period_ns, pulse_ns);
}

// DAC Stuff
static const struct device *dac_dev = DEVICE_DT_GET(DT_NODELABEL(dac1));

/* Configuration for the two DAC channels */
// static const struct dac_channel_cfg dac_ch1_cfg = {
// 	.channel_id = 1,
// 	.resolution = 12};
static const struct dac_channel_cfg dac_ch2_cfg = {
	.channel_id = 2,
	.resolution = 12};

void set_motor_current(uint16_t vref_mv)
{
	/* STM32 DAC is 12-bit (0-4095). Assuming VDD is 3.3V (3300mV) */
	uint32_t dac_value = (vref_mv * 4095) / 3300;

	// dac_write_value(dac_dev, 1, dac_value);
	dac_write_value(dac_dev, 2, dac_value);
}

// DRV Control Pins
static const struct gpio_dt_spec sleep_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_sleep), gpios);
static const struct gpio_dt_spec fault_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_fault), gpios);
/* Callback structure for fault interrupts */
static struct gpio_callback fault_cb_data;

void fault_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    /* Handle driver fault (e.g., stop PWM, log error) */
    printk("DRV8424 Fault Detected!\n");
}

// LED Blinking Thread
void gpio_led_thread_entry(void *p1, void *p2, void *p3)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led))
	{
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return;
	}

	while (1)
	{
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
			return;
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


	/* Initialize PWM pins to 0% duty cycle (off) to prevent unwanted motor movement */
	pwm_set_pulse_dt(&dir_pin, 0);


	init_motor_driver();
	// /* Example: Run Motor A as DC at 50% Duty Cycle */
	// set_dc_motor(&drv_ain1, &drv_ain2, drv_ain1.period / 2);

	// /* Example: Run Motor B as DC at 25% Duty Cycle */
	// set_dc_motor(&drv_bin1, &drv_bin2, drv_bin1.period / 4);

	if (!device_is_ready(dac_dev)) {
		printk("DAC device not ready\n");
		return 0;
	}

	dac_channel_setup(dac_dev, &dac_ch2_cfg);

	set_motor_current(2000); /* Set motor currents to ~1.5V/1.32 = 1136mA */

	if (!gpio_is_ready_dt(&sleep_spec) || !gpio_is_ready_dt(&fault_spec)) {
		printk("GPIO devices not ready\n");
		return 0;
	}

    /* Configure Pins */
    gpio_pin_configure_dt(&sleep_spec, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&fault_spec, GPIO_INPUT);

    /* Setup Interrupt for nFAULT */
    gpio_pin_interrupt_configure_dt(&fault_spec, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&fault_cb_data, fault_handler, BIT(fault_spec.pin));
    gpio_add_callback(fault_spec.port, &fault_cb_data);

	k_msleep(500); /* Short delay to ensure everything is initialized */
    /* Wake up the driver (nSLEEP High) */
    gpio_pin_set_dt(&sleep_spec, 1);
    
    /* Wait for device wakeup time (approx 1ms) */
    k_msleep(1);
 

	printk("System Ready! LED should be blinking\n");

	/* Start motor at 1000 Hz (1000 steps per second) */
    set_motor_speed(1000);

	/* Test sequence: continuous half-stepping only */
	while (1)
	{
		int fault_active = gpio_pin_get_dt(&fault_spec);
		if (fault_active) {
			printk("Fault detected, stopping motor\n");
			continue;
		}
		k_sleep(K_MSEC(2000));
		printk("Toggling direction\n");
        gpio_pin_toggle_dt(&dir_pin);
	}

	return 0;
}
