#ifndef AS5047P_H
#define AS5047P_H

#include <zephyr/types.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/sensor.h>


/* AS5047P Register Addresses */
#define AS5047P_REG_ERRFL 0x0001
#define AS5047P_REG_SETTINGS1 0x0018
#define AS5047P_REG_SETTINGS2 0x0019
#define AS5047P_REG_MAG 0x3FFD
#define AS5047P_REG_DIAAGC 0x3FFC
#define AS5047P_REG_ANGLEUNC 0x3FFE
#define AS5047P_REG_ANGLECOM 0x3FFF
#define AS5047P_REG_ABIBIN 0x001A
#define AS5047P_REG_ABIRES 0x001B
#define AS5047P_REG_ZPOSM 0x001C
#define AS5047P_REG_ZPOSL 0x001D
#define AS5047P_REG_SETTINGS3 0x001E
#define AS5047P_REG_PROGOPT 0x001F

/* Custom sensor channels */
#define AS5047P_CHAN_RAW SENSOR_CHAN_PRIV_START
#define AS5047P_CHAN_MAGNITUDE (SENSOR_CHAN_PRIV_START + 1)
#define AS5047P_CHAN_AMPLITUDE (SENSOR_CHAN_PRIV_START + 2)

/* SETTINGS1 Register Bit Fields */
#define AS5047P_SETTINGS1_UVW_POS 0xC000 /* UVW_AB selection */
#define AS5047P_SETTINGS1_PWM_ON 0x2000  /* PWM output enable */
#define AS5047P_SETTINGS1_DAECDIS 0x1000 /* Dynamic angle error compensation disable */
#define AS5047P_SETTINGS1_SPITH 0x0F00   /* SPI threshold */
#define AS5047P_SETTINGS1_DIR 0x0080     /* Direction */
#define AS5047P_SETTINGS1_HYST 0x0030    /* Hysteresis */
#define AS5047P_SETTINGS1_ABIRES 0x0003  /* ABI resolution */

/* SETTINGS2 Register Bit Fields */
#define AS5047P_SETTINGS2_UVW_ABI 0x2000 /* UVW_ABI selection */
#define AS5047P_SETTINGS2_DCT 0x1000     /* Fast mode */
#define AS5047P_SETTINGS2_SDA 0x0800     /* SDA enable */
#define AS5047P_SETTINGS2_PWMF 0x0400    /* PWM frequency */
#define AS5047P_SETTINGS2_DAE 0x0200     /* Dynamic angle error compensation */
#define AS5047P_SETTINGS2_WD 0x0100      /* Watchdog */
#define AS5047P_SETTINGS2_TPOLE 0x00C0   /* Turn-on programming */
#define AS5047P_SETTINGS2_PWMMOD 0x0038  /* PWM modulation */
#define AS5047P_SETTINGS2_OTP_EN 0x0007  /* OTP programming enable */

/* DIAAGC Register Bit Fields */
#define AS5047P_DIAAGC_OCF 0x8000       /* Offset compensation finished */
#define AS5047P_DIAAGC_COF 0x4000       /* Cordic overflow */
#define AS5047P_DIAAGC_COMP_LOW 0x2000  /* Compensation low */
#define AS5047P_DIAAGC_COMP_HIGH 0x1000 /* Compensation high */
#define AS5047P_DIAAGC_MAGL 0x0800      /* Magnetic field strength low */
#define AS5047P_DIAAGC_MAGH 0x0400      /* Magnetic field strength high */
#define AS5047P_DIAAGC_AGC 0x00FF       /* Automatic gain control */

/* Error flag definitions */
#define AS5047P_ERR_FIELD_TOO_LOW 0x08      /* Magnetic field too weak */
#define AS5047P_ERR_FIELD_TOO_HIGH 0x04     /* Magnetic field too strong */
#define AS5047P_ERR_CORDIC_OVERFLOW 0x10    /* Cordic overflow */
#define AS5047P_ERR_OFFSET_IN_PROGRESS 0x80 /* Offset compensation in progress */

/* SPI Command Bits */
#define AS5047P_CMD_READ 0x4000
#define AS5047P_CMD_WRITE 0x0000
#define AS5047P_CMD_PARITY_MASK 0x8000

#define AS5047P_MAX_SPI_FREQ 10000000 /* 10 MHz max */
#define AS5047P_POWER_UP_DELAY_MS 10  /* Power-up stabilization delay */

/* Function Prototypes */
uint16_t as5047p_read_reg(const struct device *spi_dev,
                        const struct spi_config *cfg,
                        const struct gpio_dt_spec *cs,  
                        uint16_t reg);

float as5047p_get_angle_degrees(const struct device *spi_dev,
                            const struct spi_config *cfg,
                            const struct gpio_dt_spec *cs);

int as5047p_write_reg(const struct device *spi_dev,
                      const struct spi_config *cfg,
                      const struct gpio_dt_spec *cs,
                      uint16_t reg,
                      uint16_t data);

int as5047p_set_abi_4096(const struct device *spi_dev, 
                         const struct spi_config *cfg,
                         const struct gpio_dt_spec *cs);

void as5047p_dump_registers(const struct device *spi_dev,
                            const struct spi_config *cfg,
                            const struct gpio_dt_spec *cs);

#endif /* AS5047P_H */