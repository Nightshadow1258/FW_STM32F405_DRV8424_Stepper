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

#include "as5047p.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define SPI1_NODE DT_NODELABEL(spi1)
static const struct device *spi1_dev = DEVICE_DT_GET(SPI1_NODE);

static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(as5047p_cs), gpios);

static struct spi_config spi_cfg = {
    .frequency = 5000000, // 500 KHz
    .operation = SPI_OP_MODE_MASTER |
                 SPI_WORD_SET(8) |
                 SPI_TRANSFER_MSB |
                 SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL, // <--- If this is set, CS is handled by the hardware of the MCU, NULL not connected manual mode
};

// ABI Interface
/* Get the devices from Devicetree */
const struct device *qdec_dev = DEVICE_DT_GET(DT_NODELABEL(qdec));
static const struct gpio_dt_spec index_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(index_pin), gpios);
static struct gpio_callback index_cb_data;

/* Callback function for Index pulse */
void index_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    /* Reset the timer counter when index pulse is detected */
    /* Note: Direct register access is often fastest for this */
    TIM4->CNT = 0;
}

void setup_encoder(void)
{
    if (!device_is_ready(qdec_dev))
    {
        return;
    }

    /* Setup Index Interrupt */
    gpio_pin_configure_dt(&index_gpio, GPIO_INPUT);
    gpio_init_callback(&index_cb_data, index_handler, BIT(index_gpio.pin));
    gpio_add_callback(index_gpio.port, &index_cb_data);
    gpio_pin_interrupt_configure_dt(&index_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

void read_encoder(void)
{
    struct sensor_value val;
    sensor_sample_fetch(qdec_dev);
    sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &val);

    /* val.val1 contains the current ticks */
    // LOG_INF("Encoder Ticks: %d", val.val1);
}

// Measurement

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

//
// Trigger pin for starting the measurement
static const struct gpio_dt_spec trigger_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(meas_trigger), gpios);

static struct gpio_callback trigger_cb_data;
static K_SEM_DEFINE(meas_sem, 0, 1);

static uint32_t last_trigger_time = 0;
#define DEBOUNCE_TIMEOUT_MS 100
void trigger_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    uint32_t now = k_uptime_get_32();

    // Only process the interrupt if enough time has passed
    if ((now - last_trigger_time) > DEBOUNCE_TIMEOUT_MS)
    {
        last_trigger_time = now;
        k_sem_give(&meas_sem);
    }
}

/* Data Structure for high-speed capture */
struct sensor_test_sample
{
    uint32_t timestamp;
    int32_t abi_ticks;
    uint16_t spi_raw;
};

/* Buffer configuration */
#define MAX_SAMPLES 100 // max ist around 8000
static struct sensor_test_sample sample_buffer[MAX_SAMPLES];

void dump_characterization_data(void)
{
    // Crucial: Wait for the measurement thread to be 100% idle
    k_msleep(100);

    // Get the hardware clock frequency
    uint32_t hw_freq = sys_clock_hw_cycles_per_sec();

    // Capture the first timestamp as the zero reference
    uint32_t start_cycles = sample_buffer[0].timestamp;

    // Use a simpler header
    printk("idx,ts [ms],Angle from ABI [°],Angle from SPI [°]\n");

    for (uint32_t i = 0; i < MAX_SAMPLES; i++)
    {
        // 1. Calculate relative cycles from the start
        uint32_t rel_cycles = sample_buffer[i].timestamp - start_cycles;

    /* Calculate ms as float then round to nearest integer */
        float rel_ms_float = (float)rel_cycles / (hw_freq / 1000.0f);
        uint32_t rel_ms_int = (uint32_t)(rel_ms_float + 0.5f);

        // Print and then wait for the UART to clear
        printk("%u,%u,%d,%f\n",
               i,
               rel_ms_int,
               sample_buffer[i].abi_ticks,
               (double)sample_buffer[i].spi_raw * (360.0 / 16384.0));

        // Slow down the dump significantly to ensure no data loss
        // 5ms delay per line is safe for 115200 baud
        k_msleep(10);
    }
}

/* Thread function */
void measurement_thread_entry(void *p1, void *p2, void *p3)
{
    struct k_timer timer;
    k_timer_init(&timer, NULL, NULL);

    while (1)
    {
        /* Wait for PB3 trigger */
        k_sem_take(&meas_sem, K_FOREVER);

        /* Optional 100ms startup delay */
        // k_msleep(100);
        printk("Measurement triggered! Starting data capture...\n");
        uint32_t start_time = k_uptime_get_32();

        /* Start the interval timer for 50ms */
        k_timer_start(&timer, K_NO_WAIT, K_MSEC(10));

        /* Collect exactly MAX_SAMPLES */
        for (int i = 0; i < MAX_SAMPLES; i++)
        {
            k_timer_status_sync(&timer);
            /* 1. Fetch ABI Ticks */
            struct sensor_value qdec_value = {0};
            if (sensor_sample_fetch(qdec_dev) == 0)
            {
                sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &qdec_value);
            }

            /* 2. Fetch SPI Raw Angle */
            uint16_t raw_reg = as5047p_read_reg(spi1_dev, &spi_cfg, &cs_gpio, AS5047P_REG_ANGLECOM);

            /* 3. Store to RAM using the loop index 'i' */
            sample_buffer[i].timestamp = k_cycle_get_32();
            sample_buffer[i].abi_ticks = qdec_value.val1;
            sample_buffer[i].spi_raw = raw_reg & 0x3FFF;

            /* Wait for the remainder of the 10ms interval */

        }
        k_timer_stop(&timer);

        uint32_t end_time = k_uptime_get_32();
        printk("Capture End. Duration: %u ms\n", (end_time - start_time));
        printk("Data capture complete! Dumping data...\n");
        dump_characterization_data();
        /* Thread automatically loops back to k_sem_take and sleeps */
    }
}

K_THREAD_DEFINE(meas_thread_id, 2048, measurement_thread_entry, NULL, NULL, NULL, 2, 0, 0);
/////

// PWM LED Section
#define PWM_THREAD_STACK_SIZE 512
#define PWM_THREAD_PRIORITY 10 // Positive number = lower priority than main (0)
#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

void pwm_led_thread_entry(void *p1, void *p2, void *p3)
{
    uint32_t max_period = MAX_PERIOD;
    uint32_t period;
    uint8_t dir = 0U;

    if (!pwm_is_ready_dt(&pwm_led0))
    {
        return;
    }

    /* Calibration logic moved to thread */
    while (pwm_set_dt(&pwm_led0, max_period, max_period / 2U))
    {
        max_period /= 2U;
        if (max_period < (4U * MIN_PERIOD))
        {
            return;
        }
    }

    period = max_period;

    while (1)
    {
        /* Set PWM duty cycle */
        pwm_set_dt(&pwm_led0, period, period / 2U);

        /* Update period for next iteration */
        period = dir ? (period * 2U) : (period / 2U);

        if (period > max_period)
        {
            period = max_period / 2U;
            dir = 0U;
        }
        else if (period < MIN_PERIOD)
        {
            period = MIN_PERIOD * 2U;
            dir = 1U;
        }

        /* Thread sleeps, yielding to high-prio sensor tasks */
        k_sleep(K_SECONDS(4));
    }
}
/* Define the low-priority thread */
K_THREAD_DEFINE(pwm_led_tid, PWM_THREAD_STACK_SIZE,
                pwm_led_thread_entry, NULL, NULL, NULL,
                PWM_THREAD_PRIORITY, 0, 0);

int main(void)
{
    if (!gpio_is_ready_dt(&cs_gpio))
    {
        LOG_ERR("CS GPIO device not ready");
        return -ENODEV;
    }
    /* Configure as output and set to High (Inactive) */
    gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(spi1_dev))
    {
        return -ENODEV;
    }
    if (!device_is_ready(spi1_dev))
    {
        return -ENODEV;
    }

    // Setup the encoder pins for the ABI Interface
    setup_encoder();
    // Setup the encoder pins for the ABI Interface
    setup_encoder();

    // Check register settings
    as5047p_dump_registers(spi1_dev, &spi_cfg, &cs_gpio);
    as5047p_set_abi_4096(spi1_dev, &spi_cfg, &cs_gpio);
    as5047p_dump_registers(spi1_dev, &spi_cfg, &cs_gpio);
    k_msleep(100);

    // Interrupt setup for measurement trigger
    gpio_pin_configure_dt(&trigger_gpio, GPIO_INPUT);

    /* Configure interrupt for Rising Edge (Low to High) */
    gpio_pin_interrupt_configure_dt(&trigger_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    /* Attach the callback */
    gpio_init_callback(&trigger_cb_data, trigger_handler, BIT(trigger_gpio.pin));
    gpio_add_callback(trigger_gpio.port, &trigger_cb_data);

    printk("System Ready. Trigger enabled.\n");

    while (1)
    {
        unsigned int count = k_sem_count_get(&meas_sem);
        // Avoid frequent printf during active measurements
        if (count > 0)
        {
            printk("Pending measurements: %u\n", count);
        }
        k_msleep(2000);
    }

    return 0;
    return 0;
}
