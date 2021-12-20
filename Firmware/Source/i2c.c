/**
 * @file i2c.c
 * @author Tom Klijn
 * @brief 
 * @version 0.1
 * @date 2021-12-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* ========================== include files =========================== */
#include <stdint.h>
#include <stdio.h>
#include "gd32f10x.h"
#include "systick.h"
#include "i2c.h"

/* ============================ constants ============================= */
#define I2C0_SLAVE_ADDRESS7 (0x78)

/* =========================== Typedefs =============================== */

/* ======================== global variables ========================== */

/* ==================== function prototypes =========================== */

/* ============================ functions ============================= */

void i2c_config()
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);

    /* I2C0 and I2C1 GPIO ports */
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void i2c_busScan()
{
    uint8_t maxAdd = (0xFF >> 1); //7bit max

    for (uint8_t add = 1; add <= maxAdd; add++)
    {
        /* wait until I2C bus is idle */
        while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
            ;
        /* send a start condition to I2C bus */
        i2c_start_on_bus(I2C0);
        /* wait until SBSEND bit is set */
        while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
            ;

        /* send slave address to I2C bus */
        i2c_master_addressing(I2C0, add, I2C_TRANSMITTER);
        
        /* wait until ADDSEND bit is set */
        // while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        delay_1ms(10);
        if (i2c_flag_get(I2C0, I2C_FLAG_AERR))
        {
            //No ACK error received for Address
            printf(".");
            i2c_flag_clear(I2C0, I2C_FLAG_AERR);
        }
        else
        {
            printf(" %#02X ", add);
        }

        /* clear ADDSEND bit */
        i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(I2C0);
        while (I2C_CTL0(I2C0) & 0x0200)
            ;
    }
    printf("\n");
}
