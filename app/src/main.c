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
static const struct pwm_dt_spec drv_ain1 = PWM_DT_SPEC_GET(DT_NODELABEL(ain1));
static const struct pwm_dt_spec drv_ain2 = PWM_DT_SPEC_GET(DT_NODELABEL(ain2));
static const struct pwm_dt_spec drv_bin1 = PWM_DT_SPEC_GET(DT_NODELABEL(bin1));
static const struct pwm_dt_spec drv_bin2 = PWM_DT_SPEC_GET(DT_NODELABEL(bin2));

/* Helper for DC Motor Control */
void set_dc_motor(const struct pwm_dt_spec *p1, const struct pwm_dt_spec *p2, uint32_t pulse)
{
	pwm_set_pulse_dt(p1, pulse);
	pwm_set_pulse_dt(p2, 0);
}

/* Helper for Stepper Motor Control */
/* Full Step Sequence Table (AIN1, AIN2, BIN1, BIN2) */
static const uint8_t step_table[4][4] = {
	{1, 0, 1, 0}, /* Step 0: A+ B+ */
	{0, 1, 1, 0}, /* Step 1: A- B+ */
	{0, 1, 0, 1}, /* Step 2: A- B- */
	{1, 0, 0, 1}  /* Step 3: A+ B- */
};

void apply_step(int step_index, uint32_t pulse_width)
{
	pwm_set_pulse_dt(&drv_ain1, step_table[step_index][0] ? pulse_width : 0);
	pwm_set_pulse_dt(&drv_ain2, step_table[step_index][1] ? pulse_width : 0);
	pwm_set_pulse_dt(&drv_bin1, step_table[step_index][2] ? pulse_width : 0);
	pwm_set_pulse_dt(&drv_bin2, step_table[step_index][3] ? pulse_width : 0);
}

// DAC Stuff
static const struct device *dac_dev = DEVICE_DT_GET(DT_NODELABEL(dac1));

/* Configuration for the two DAC channels */
static const struct dac_channel_cfg dac_ch1_cfg = {
	.channel_id = 1,
	.resolution = 12};
static const struct dac_channel_cfg dac_ch2_cfg = {
	.channel_id = 2,
	.resolution = 12};

void set_motor_current(uint16_t vref_mv)
{
	/* STM32 DAC is 12-bit (0-4095). Assuming VDD is 3.3V (3300mV) */
	uint32_t dac_value = (vref_mv * 4095) / 3300;

	dac_write_value(dac_dev, 1, dac_value);
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
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}

	while (1)
	{
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
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
	/* DC Motor Driver Control*/
	if (!pwm_is_ready_dt(&drv_ain1) || !pwm_is_ready_dt(&drv_ain2) ||
	    !pwm_is_ready_dt(&drv_bin1) || !pwm_is_ready_dt(&drv_bin2)) {
		printk("PWM devices not ready\n");
		return 0;
	}

	// /* Example: Run Motor A as DC at 50% Duty Cycle */
	// set_dc_motor(&drv_ain1, &drv_ain2, drv_ain1.period / 2);

	// /* Example: Run Motor B as DC at 25% Duty Cycle */
	// set_dc_motor(&drv_bin1, &drv_bin2, drv_bin1.period / 4);

	if (!device_is_ready(dac_dev)) {
		printk("DAC device not ready\n");
		return 0;
	}

	dac_channel_setup(dac_dev, &dac_ch1_cfg);
	dac_channel_setup(dac_dev, &dac_ch2_cfg);

	set_motor_current(1500); /* Set motor currents to ~1.5A */

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

	int current_step = 0;
	uint32_t duty_cycle = drv_ain1.period; /* 100% duty cycle for full power */
	printk("Duty Cycle: %lu ns (100%%)\n", (unsigned long)duty_cycle);

	static int step_delay = 200; // Start at 200ms

	while (1)
	{
		int fault_active = gpio_pin_get_dt(&fault_spec);
		if (fault_active) {
			printk("Fault detected, stopping motor\n");
		}

		apply_step(current_step, duty_cycle);

		current_step = (current_step + 1) % 4;
		/* Delay determines speed - start slow and ramp up */

		if (step_delay > 50) step_delay -= 1; // Ramp down to 50ms -> 1.8 degree per step at 50ms per step = 36 degree per second
		//printk("Current Step: %d, Step Delay: %d us\n", current_step, step_delay, );
		k_usleep(step_delay);
	}

	return 0;
}
