/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT ams_as5047p


#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

/* AS5047P Register Addresses */
#define AS5047P_REG_ERRFL   		 0x0001
#define AS5047P_REG_SETTINGS1        0x0018
#define AS5047P_REG_SETTINGS2        0x0019
#define AS5047P_REG_MAG              0x3FFD
#define AS5047P_REG_DIAAGC           0x3FFC
#define AS5047P_REG_ANGLEUNC         0x3FFE
#define AS5047P_REG_ANGLECOM         0x3FFF
#define AS5047P_REG_ABIBIN           0x001A
#define AS5047P_REG_ABIRES           0x001B
#define AS5047P_REG_ZPOSM            0x001C	
#define AS5047P_REG_ZPOSL            0x001D
#define AS5047P_REG_SETTINGS3        0x001E
#define AS5047P_REG_PROGOPT          0x001F

/* Custom sensor channels */
#define AS5047P_CHAN_RAW            SENSOR_CHAN_PRIV_START
#define AS5047P_CHAN_MAGNITUDE       (SENSOR_CHAN_PRIV_START + 1)
#define AS5047P_CHAN_AMPLITUDE       (SENSOR_CHAN_PRIV_START + 2)

/* SETTINGS1 Register Bit Fields */
#define AS5047P_SETTINGS1_UVW_POS    0xC000  /* UVW_AB selection */
#define AS5047P_SETTINGS1_PWM_ON     0x2000  /* PWM output enable */
#define AS5047P_SETTINGS1_DAECDIS    0x1000  /* Dynamic angle error compensation disable */
#define AS5047P_SETTINGS1_SPITH      0x0F00  /* SPI threshold */
#define AS5047P_SETTINGS1_DIR        0x0080  /* Direction */
#define AS5047P_SETTINGS1_HYST       0x0030  /* Hysteresis */
#define AS5047P_SETTINGS1_ABIRES     0x0003  /* ABI resolution */

/* SETTINGS2 Register Bit Fields */
#define AS5047P_SETTINGS2_UVW_ABI    0x2000  /* UVW_ABI selection */
#define AS5047P_SETTINGS2_DCT        0x1000  /* Fast mode */
#define AS5047P_SETTINGS2_SDA        0x0800  /* SDA enable */
#define AS5047P_SETTINGS2_PWMF       0x0400  /* PWM frequency */
#define AS5047P_SETTINGS2_DAE        0x0200  /* Dynamic angle error compensation */
#define AS5047P_SETTINGS2_WD         0x0100  /* Watchdog */
#define AS5047P_SETTINGS2_TPOLE      0x00C0  /* Turn-on programming */
#define AS5047P_SETTINGS2_PWMMOD     0x0038  /* PWM modulation */
#define AS5047P_SETTINGS2_OTP_EN     0x0007  /* OTP programming enable */

/* DIAAGC Register Bit Fields */
#define AS5047P_DIAAGC_OCF           0x8000  /* Offset compensation finished */
#define AS5047P_DIAAGC_COF           0x4000  /* Cordic overflow */
#define AS5047P_DIAAGC_COMP_LOW      0x2000  /* Compensation low */
#define AS5047P_DIAAGC_COMP_HIGH     0x1000  /* Compensation high */
#define AS5047P_DIAAGC_MAGL          0x0800  /* Magnetic field strength low */
#define AS5047P_DIAAGC_MAGH          0x0400  /* Magnetic field strength high */
#define AS5047P_DIAAGC_AGC           0x00FF  /* Automatic gain control */

/* Error flag definitions */
#define AS5047P_ERR_FIELD_TOO_LOW    0x08    /* Magnetic field too weak */
#define AS5047P_ERR_FIELD_TOO_HIGH   0x04    /* Magnetic field too strong */
#define AS5047P_ERR_CORDIC_OVERFLOW  0x10    /* Cordic overflow */
#define AS5047P_ERR_OFFSET_IN_PROGRESS 0x80   /* Offset compensation in progress */

/* SPI Command Bits */
#define AS5047P_CMD_READ             0x4000
#define AS5047P_CMD_WRITE            0x0000
#define AS5047P_CMD_PARITY_MASK      0x8000

#define AS5047P_MAX_SPI_FREQ         10000000  /* 10 MHz max */
#define AS5047P_POWER_UP_DELAY_MS    10        /* Power-up stabilization delay */

struct as5047p_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset_gpio;
};

struct as5047p_data {
	uint16_t angle;
	uint16_t raw_angle;
	uint16_t magnitude;
	uint8_t agc_value;
	uint8_t error_flags;
};

LOG_MODULE_REGISTER(as5047p, CONFIG_SENSOR_LOG_LEVEL);

static int as5047p_compute_parity(uint16_t value)
{
	int parity = 0;
	
	/* Compute parity for bits 0-14 only (15 bits total) */
	for (int i = 0; i < 15; i++) {
		if (value & (1 << i)) {
			parity ^= 1;
		}
	}
	
	return parity;
}

static int as5047p_spi_write(const struct device *dev, uint16_t addr, uint16_t data)
{
	const struct as5047p_config *config = dev->config;
	uint16_t cmd = (addr & 0x3FFF) | AS5047P_CMD_WRITE;
	uint16_t tx_buf[2], rx_buf[2];
	struct spi_buf tx_bufs = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx_bufs = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx_bufs, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_bufs, .count = 1 };

	/* Add parity bit */
	if (as5047p_compute_parity(cmd)) {
		cmd |= AS5047P_CMD_PARITY_MASK;
	}

	/* First transfer: command + dummy data */
	tx_buf[0] = sys_cpu_to_be16(cmd);
	tx_buf[1] = sys_cpu_to_be16(data);

	if (spi_transceive_dt(&config->bus, &tx_set, &rx_set) != 0) {
		LOG_ERR("SPI write failed");
		return -EIO;
	}

	return 0;
}

static int as5047p_spi_read(const struct device *dev, uint16_t addr, uint16_t *data)
{
	const struct as5047p_config *config = dev->config;
	uint16_t cmd = (addr & 0x3FFF) | AS5047P_CMD_READ;
	uint16_t tx_buf[2], rx_buf[2];
	struct spi_buf tx_bufs = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf rx_bufs = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx_bufs, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_bufs, .count = 1 };

	/* Add parity bit */
	if (as5047p_compute_parity(cmd)) {
		cmd |= AS5047P_CMD_PARITY_MASK;
	}

	/* First transfer: command + dummy data */
	tx_buf[0] = sys_cpu_to_be16(cmd);
	tx_buf[1] = 0x0000;

	if (spi_transceive_dt(&config->bus, &tx_set, &rx_set) != 0) {
		LOG_ERR("SPI read failed");
		return -EIO;
	}

	*data = sys_be16_to_cpu(rx_buf[1]);

	return 0;
}

static int as5047p_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as5047p_data *data = dev->data;
	int ret;
	uint16_t reg_value;

	ARG_UNUSED(chan);

	/* Read angle (compensated) */
	ret = as5047p_spi_read(dev, AS5047P_REG_ANGLECOM, &reg_value);
	if (ret < 0) {
		return ret;
	}
	data->angle = reg_value & 0x3FFF;  /* 14-bit angle */

	/* Read raw angle */
	ret = as5047p_spi_read(dev, AS5047P_REG_ANGLEUNC, &reg_value);
	if (ret < 0) {
		return ret;
	}
	data->raw_angle = reg_value & 0x3FFF;  /* 14-bit raw angle */

	/* Read diagnostics and AGC */
	ret = as5047p_spi_read(dev, AS5047P_REG_DIAAGC, &reg_value);
	if (ret < 0) {
		return ret;
	}
	data->agc_value = reg_value & 0x00FF;
	data->error_flags = (reg_value >> 8) & 0xFF;

	/* Read magnitude */
	ret = as5047p_spi_read(dev, AS5047P_REG_MAG, &reg_value);
	if (ret < 0) {
		return ret;
	}
	data->magnitude = reg_value & 0x3FFF;  /* 14-bit magnitude */

	return 0;
}

static int as5047p_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct as5047p_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		/* Convert 14-bit angle to degrees */
		val->val1 = (data->angle * 360) / 16384;
		val->val2 = ((data->angle * 360000) % 16384) * 1000 / 16384;
		break;

	case SENSOR_CHAN_RAW:
		/* Return raw 14-bit angle */
		val->val1 = data->angle;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_MAGNITUDE:
		/* Return 14-bit magnitude */
		val->val1 = data->magnitude;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_AMPLITUDE:
		/* Return AGC value as proxy for signal strength */
		val->val1 = data->agc_value;
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int as5047p_init(const struct device *dev)
{
	const struct as5047p_config *config = dev->config;
	int ret;
	uint16_t settings1, settings2;

	LOG_INF("Initializing AS5047P magnetic encoder");

	/* Configure SPI bus */
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Reset if GPIO is available */
	if (config->reset_gpio.port) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		k_msleep(1);
		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_msleep(AS5047P_POWER_UP_DELAY_MS);
	}

	/* Read current settings */
	ret = as5047p_spi_read(dev, AS5047P_REG_SETTINGS1, &settings1);
	if (ret < 0) {
		LOG_ERR("Failed to read SETTINGS1 register");
		return ret;
	}

	ret = as5047p_spi_read(dev, AS5047P_REG_SETTINGS2, &settings2);
	if (ret < 0) {
		LOG_ERR("Failed to read SETTINGS2 register");
		return ret;
	}

	LOG_DBG("Initial SETTINGS1: 0x%04X, SETTINGS2: 0x%04X", 
		settings1, settings2);

	/* Configure default settings if needed */
	if ((settings1 & 0xFF) == 0xFF) {
		/* Device appears to be in factory reset state */
		settings1 = (0 << 14) |  /* UVW_POS = 0 */
			    (0 << 13) |  /* PWM_ON = 0 */
			    (0 << 12) |  /* DAECDIS = 0 */
			    (0 << 8)   |  /* SPITH = 0 */
			    (0 << 7)   |  /* DIR = 0 (normal) */
			    (1 << 4)   |  /* HYST = 1 */
			    (2 << 0);    /* ABIRES = 2 (1024 CPR) */

		ret = as5047p_spi_write(dev, AS5047P_REG_SETTINGS1, settings1);
		if (ret < 0) {
			LOG_ERR("Failed to write SETTINGS1 register");
			return ret;
		}
	}

	LOG_INF("AS5047P initialized successfully");
	return 0;
}

int as5047p_test_link(const struct device *dev)
{
    const struct as5047p_config *config = dev->config;
    uint16_t cmd = 0xC001; // Read Error Flag
    uint16_t rx_data = 0;

    struct spi_buf tx_buf = { .buf = &cmd, .len = 2 };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = &rx_data, .len = 2 };
    struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

    int err = spi_transceive_dt(&config->bus, &tx_bufs, &rx_bufs);
    if (err) {
        LOG_ERR("SPI transfer failed: %d", err);
        return err;
    }

    /* Second transfer needed to get the result of the previous command */
    spi_transceive_dt(&config->bus, &tx_bufs, &rx_bufs);

    LOG_INF("Raw SPI Response: 0x%04x", rx_data);
    return 0;
}


static const struct sensor_driver_api as5047p_driver_api = {
	.sample_fetch = as5047p_sample_fetch,
	.channel_get = as5047p_channel_get,
};

#define AS5047P_INIT(inst) \
	static struct as5047p_data as5047p_data_##inst; \
	static const struct as5047p_config as5047p_config_##inst = { \
		.spi = SPI_DT_SPEC_INST_GET(inst, \
				SPI_WORD_SET(16) | SPI_TRANSFER_MSB), \
	}; \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, as5047p_init, NULL, \
				 &as5047p_data_##inst, &as5047p_config_##inst, \
				 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				 &as5047p_api);

DT_INST_FOREACH_STATUS_OKAY(AS5047P_INST)
