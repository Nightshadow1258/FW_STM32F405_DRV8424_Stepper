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

// USB CDC ACM for console output
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

// Shell for interactive commands (optional)
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

// LED Stuff
#define SLEEP_TIME_MS 1000
#define GPIO_LED_THREAD_STACK_SIZE 1024
#define GPIO_LED_THREAD_PRIORITY 3
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Motor control stuff
static const struct gpio_dt_spec enable_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_enable), gpios);
static const struct gpio_dt_spec dir_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_dir), gpios);
static const struct gpio_dt_spec mode1_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_mode1), gpios);
static const struct gpio_dt_spec mode0_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(drv_mode0), gpios);

static const struct pwm_dt_spec step_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(drv_step));

void init_motor_driver(void)
{
	/* Configure GPIOs as output */

	gpio_pin_configure_dt(&enable_pin, GPIO_OUTPUT_INACTIVE); /* nENABLE is active LOW, so we set it to LOW to enable the driver */
	gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);

	/* Set M1 to High Impedance by configuring it as an Input */
	/* Ensure no internal pull-up/pull-down is active to allow true Hi-Z */
	gpio_pin_configure_dt(&mode1_pin, GPIO_INPUT);
	/* M0 must be High for 1/128 mode when M1 is Hi-Z */
	gpio_pin_configure_dt(&mode0_pin, GPIO_INPUT);
	gpio_pin_configure_dt(&mode0_pin, GPIO_OUTPUT_ACTIVE);

	/* Both M0 and M1 are now High, setting 1/16 microstepping */
}

void set_motor_speed(uint32_t frequency_hz)
{
	if (frequency_hz == 0)
	{
		pwm_set_pulse_dt(&step_pwm, 0);
		return;
	}

	uint32_t period_ns = 1000000000ULL / frequency_hz;
	uint32_t pulse_ns = period_ns / 2;

	pwm_set_dt(&step_pwm, period_ns, pulse_ns);
}

#define TARGET_FREQ 25600 /* 1 RPS */

#define START_FREQ 100	  /* Start at 100 Hz instead of 500+ Hz */
#define RAMP_DURATION 500 /* 500ms ramp instead of 100ms */
#define RAMP_STEPS 100	  /* More granular steps */

#define TOTAL_STEPS 51200
#define RAMP_STEPS_COUNT 100
#define STEP_INTERVAL_MS 5

void move_exact_rotations(uint32_t target_hz)
{
	uint32_t steps_taken = 0;
	uint32_t current_hz = START_FREQ;
	uint32_t ramp_inc = (target_hz - START_FREQ) / RAMP_STEPS_COUNT;

	/* 1. Ramp Up */
	for (int i = 0; i < RAMP_STEPS_COUNT; i++)
	{
		set_motor_speed(current_hz);
		/* Estimate steps: freq * time_in_seconds */
		steps_taken += (current_hz * STEP_INTERVAL_MS) / 1000;
		current_hz += ramp_inc;
		k_msleep(STEP_INTERVAL_MS);
	}

	/* 2. Constant Speed */
	set_motor_speed(target_hz);
	/* Calculate remaining steps needed */
	/* We subtract the ramp down steps too (which equal ramp up steps) */
	uint32_t ramp_up_steps = steps_taken;
	int32_t remaining_steps = TOTAL_STEPS - (ramp_up_steps * 2);

	if (remaining_steps > 0)
	{
		uint32_t constant_time_ms = (remaining_steps * 1000) / target_hz;
		k_msleep(constant_time_ms);
	}

	/* 3. Ramp Down */
	for (int i = 0; i < RAMP_STEPS_COUNT; i++)
	{
		current_hz -= ramp_inc;
		set_motor_speed(current_hz);
		k_msleep(STEP_INTERVAL_MS);
	}

	set_motor_speed(0);
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

void fault_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	/* Handle driver fault (e.g., stop PWM, log error) */
	printk("DRV8424 Fault Detected!\n");
}

static const struct gpio_dt_spec meas_trigger = GPIO_DT_SPEC_GET(DT_NODELABEL(meas_trigger), gpios);

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
// K_THREAD_DEFINE(gpio_led_tid, GPIO_LED_THREAD_STACK_SIZE,
// 				gpio_led_thread_entry, NULL, NULL, NULL,
//				GPIO_LED_THREAD_PRIORITY, 0, 0);

// Shell command to manually trigger the process (for testing)
static int cmd_trigger_start(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Trigger received. Starting process...");
	gpio_pin_configure_dt(&enable_pin, GPIO_OUTPUT_ACTIVE);

	/* Requirement: Set PA0 HIGH before starting */
	gpio_pin_configure_dt(&meas_trigger, GPIO_OUTPUT_ACTIVE);

	/* Start motor at 1000 Hz (1000 steps per second) */
	// set_motor_speed(25600);

	/* Test sequence: continuous half-stepping only */

	int fault_active = gpio_pin_get_dt(&fault_spec);
	if (fault_active)
	{
		printk("Fault detected, stopping motor\n");
		return -1;
	}
	printk("Sending Trigger...\n");
	gpio_pin_set_dt(&meas_trigger, 1);
	k_msleep(10); /* Brief stabilization delay */
	/* Forward 2 Rotations (Approx 2 seconds total) */
	gpio_pin_set_dt(&dir_pin, 1);
	move_exact_rotations(TARGET_FREQ);

	k_msleep(50); /* Pause at end of motion */

	/* Reverse 2 Rotations */
	gpio_pin_set_dt(&dir_pin, 0);
	move_exact_rotations(TARGET_FREQ);

	k_msleep(10);

	gpio_pin_set_dt(&meas_trigger, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&enable_pin, GPIO_OUTPUT_INACTIVE);

	k_msleep(100); /* Brief stabilization delay */

	return 0;
}

/* Register the command "start_process" */
SHELL_CMD_REGISTER(start_process, NULL, "Manually trigger the process", cmd_trigger_start);

int main(void)
{
	printk("Starting DRV8424 Stepper Motor Control Application\n");

	// Use USB CDC ACM for console output
	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL))
	{
		return 0;
	}

	/* Optional: Wait for a terminal connection */
	while (!dtr)
	{
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	/* Initialize PWM pins to 0% duty cycle (off) to prevent unwanted motor movement */
	pwm_set_pulse_dt(&step_pwm, 0);

	init_motor_driver();
	// /* Example: Run Motor A as DC at 50% Duty Cycle */
	// set_dc_motor(&drv_ain1, &drv_ain2, drv_ain1.period / 2);

	// /* Example: Run Motor B as DC at 25% Duty Cycle */
	// set_dc_motor(&drv_bin1, &drv_bin2, drv_bin1.period / 4);

	if (!device_is_ready(dac_dev))
	{
		printk("DAC device not ready\n");
		return 0;
	}

	dac_channel_setup(dac_dev, &dac_ch2_cfg);

	set_motor_current(2500); /* Set motor currents to ~2.5V/1.32 = 1900mA */

	if (!gpio_is_ready_dt(&sleep_spec) || !gpio_is_ready_dt(&fault_spec))
	{
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

	while (1)
	{
		/* Main loop can be used for background tasks or remain idle */
		k_msleep(5000);
		// printk("Main loop heartbeat...\n");
	}

	return 0;
}
