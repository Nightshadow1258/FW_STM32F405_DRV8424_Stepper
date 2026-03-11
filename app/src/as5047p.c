#include "as5047p.h"
#include <zephyr/kernel.h>

struct as5047p_reg_map
{
    uint16_t addr;
    const char *name;
};

static const struct as5047p_reg_map reg_list[] = {
    {AS5047P_REG_ERRFL, "ERRFL"},
    {AS5047P_REG_SETTINGS1, "SETTINGS1"},
    {AS5047P_REG_SETTINGS2, "SETTINGS2"},
    {AS5047P_REG_ABIBIN, "ABIBIN"},
    {AS5047P_REG_ABIRES, "ABIRES"},
    {AS5047P_REG_ZPOSM, "ZPOSM"},
    {AS5047P_REG_ZPOSL, "ZPOSL"},
    {AS5047P_REG_SETTINGS3, "SETTINGS3"},
    {AS5047P_REG_PROGOPT, "PROGOPT"},
    {AS5047P_REG_DIAAGC, "DIAAGC"},
    {AS5047P_REG_MAG, "MAG"},
    {AS5047P_REG_ANGLEUNC, "ANGLEUNC"},
    {AS5047P_REG_ANGLECOM, "ANGLECOM"}};

static uint16_t add_parity(uint16_t command)
{
    uint16_t count = 0;
    uint16_t temp = command;
    for (int i = 0; i < 16; i++)
    {
        if (temp & 0x1)
            count++;
        temp >>= 1;
    }
    return (count % 2 != 0) ? (command | 0x8000) : command;
}

uint16_t as5047p_read_reg(const struct device *spi_dev,
                          const struct spi_config *cfg,
                          const struct gpio_dt_spec *cs,
                          uint16_t reg)
{
    uint16_t command = add_parity(AS5047P_CMD_READ | (reg & 0x3FFF));
    uint8_t tx_data[2] = {(uint8_t)(command >> 8), (uint8_t)(command & 0xFF)};
    uint8_t tx_nop[2] = {0xC0, 0x00};
    uint8_t rx_buf[2];

    struct spi_buf tx = {.buf = tx_data, .len = 2};
    struct spi_buf_set tx_s = {.buffers = &tx, .count = 1};
    struct spi_buf rx = {.buf = rx_buf, .len = 2};
    struct spi_buf_set rx_s = {.buffers = &rx, .count = 1};

    /* Frame 1: Send Command */
    gpio_pin_set_dt(cs, 1);
    k_busy_wait(1);
    spi_transceive(spi_dev, cfg, &tx_s, &rx_s);
    gpio_pin_set_dt(cs, 0);

    k_busy_wait(1);

    /* Frame 2: Get Data */
    tx.buf = tx_nop;
    gpio_pin_set_dt(cs, 1);
    k_busy_wait(1);
    spi_transceive(spi_dev, cfg, &tx_s, &rx_s);
    gpio_pin_set_dt(cs, 0);

    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

float as5047p_get_angle_degrees(const struct device *spi_dev,
                                const struct spi_config *cfg,
                                const struct gpio_dt_spec *cs)
{
    /* Read the ANGLECOM register (0x3FFE) */
    uint16_t raw_val = as5047p_read_reg(spi_dev, cfg, cs, AS5047P_REG_ANGLECOM);

    /* * Mask out bits 14 (Error Flag) and 15 (Parity)
     * 0x3FFF = 0011 1111 1111 1111
     */
    uint16_t angle_data = raw_val & 0x3FFF;

    /* * Convert 14-bit value to degrees:
     * Resolution = 360.0 / 2^14 = 360.0 / 16384
     */
    float degrees = (float)angle_data * (360.0f / 16384.0f);

    return degrees;
}

int as5047p_write_reg(const struct device *spi_dev,
                      const struct spi_config *cfg,
                      const struct gpio_dt_spec *cs,
                      uint16_t reg,
                      uint16_t data)
{
    /* 1. Prepare Command Frame (Write bit is 0 at bit 14) */
    uint16_t command = add_parity(reg & 0x3FFF);
    uint8_t cmd_buf[2] = {(uint8_t)(command >> 8), (uint8_t)(command & 0xFF)};

    /* 2. Prepare Data Frame (Data bits 0:13) */
    uint16_t data_frame = add_parity(data & 0x3FFF);
    uint8_t data_buf[2] = {(uint8_t)(data_frame >> 8), (uint8_t)(data_frame & 0xFF)};

    struct spi_buf tx_cmd = {.buf = cmd_buf, .len = 2};
    struct spi_buf tx_data = {.buf = data_buf, .len = 2};

    struct spi_buf_set tx_cmd_s = {.buffers = &tx_cmd, .count = 1};
    struct spi_buf_set tx_data_s = {.buffers = &tx_data, .count = 1};

    /* Frame 1: Send Address/Write Command */
    gpio_pin_set_dt(cs, 1);
    k_busy_wait(1);
    int ret = spi_write(spi_dev, cfg, &tx_cmd_s);
    gpio_pin_set_dt(cs, 0);

    if (ret)
        return ret;

    /* Mandatory tCSn delay */
    k_busy_wait(1);

    /* Frame 2: Send Data */
    gpio_pin_set_dt(cs, 1);
    k_busy_wait(1);
    ret = spi_write(spi_dev, cfg, &tx_data_s);
    gpio_pin_set_dt(cs, 0);

    return ret;
}

void as5047p_dump_registers(const struct device *spi_dev,
                            const struct spi_config *cfg,
                            const struct gpio_dt_spec *cs)
{
    printk("\n--- AS5047P Register Dump ---\n");
    printk("%-12s | %-6s | %-6s\n", "Register", "Addr", "Value");
    printk("------------------------------\n");

    for (int i = 0; i < ARRAY_SIZE(reg_list); i++)
    {
        uint16_t value = as5047p_read_reg(spi_dev, cfg, cs, reg_list[i].addr);

        /* The data is in the lower 14 bits (0-13) */
        uint16_t data = value & 0x3FFF;
        uint8_t error = (value & 0x4000) >> 14;

        printk("%-12s | 0x%04X | 0x%04X %s\n",
               reg_list[i].name,
               reg_list[i].addr,
               data,
               error ? "(ERR)" : "");
    }
    printk("------------------------------\n\n");
}

int as5047p_set_abi_4096(const struct device *spi_dev,
                         const struct spi_config *cfg,
                         const struct gpio_dt_spec *cs)
{
    int ret;

    /* Set SETTINGS1 to 0x0021 (Factory bit = 1, ABIBIN = 1) */
    ret = as5047p_write_reg(spi_dev, cfg, cs, AS5047P_REG_SETTINGS1, 0x0021);
    if (ret)
    {
        return ret;
    }

    /* Set SETTINGS2 to 0x0000 (ABIRES = 000) */
    /* This selects 4096 Steps / 1024 Pulses when ABIBIN is 1 */
    ret = as5047p_write_reg(spi_dev, cfg, cs, AS5047P_REG_SETTINGS2, 0x0000);

    return ret;
}
