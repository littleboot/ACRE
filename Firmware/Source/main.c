/**
 * @file main.c
 * @author Tom Klijn (tomklijn@outlook.com)
 * @brief 
 * Firmware for the Absolute Capacitive Rotary Encoder (ACRE) project
 * The code generates the required signals for the ACRE to opperate and calculates the displacement angle of the 
 * reflector disk.
 * 
 * @version 0.1
 * @date 2021-12-18
 * 
 * @copyright Copyright (c) 2021
 * 
 * @TODO
 *  [X] Move sensor output signal OCTL generation from ISR
 *  [X] Configure Timer and DMA to output the signal
 *  [X] Configure ADC to sample the signal
 *  [ ] UART DMA Transmission.
 *  [X] Signal processing, to get the phase from the sampled signal
 *  [ ] Serial cli 
 *  [X] i2c display

 * @Bugs
 *  [ ] Potential bug, the DMA channel for sampling and generating the signals are not synced, so if an external ADC trig signal is missed the signals could go out of sync.
 *  [ ] I2c errors will cause endless loop, add propper timeout functionality to the driver.
 *  
 */

/* ========================== include files =========================== */
#include <stdio.h>
#include <math.h>

#include "gd32f10x.h"
#include "systick.h"
#include "i2c.h"
#include "ssd1306.h"
#include "main.h"

/* ============================ constants ============================= */
#define BIAS_BUFF_SIZE 8

#define SAMPLE_BUFF_SIZE 64

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/* ======================== global variables ========================== */
static uint32_t octlBuff[TX_BUFF_SIZE]; //Sensor output signals, NOTE! call sensor_generateOctlBuff() before use
static double cosAngLut[32];
static double sinAngLut[32];

static volatile uint16_t samples[SAMPLE_BUFF_SIZE];
static double even[32];
static double odd[32];
static double demol[32];

static double k = 1; //Frequency bin of interest 1hz

static double real = 0; //Real part of sensor signal
static double imag = 0; //Imaginary part of sensor signal

static double magnitude = 0; //Amplitude of sensor signal
static double phaseRad = 0;  //Phase in radians
static double phaseDeg = 0;  //Phase in degrees

static int quadrant = 1;
static double demolLp[32];

/* ==================== function prototypes =========================== */
void leds_config(void);
void uart_config(void);
void swd_config(void);
void sensor_config(void); // Configure sensor IO
void timer_config(void);  // configure the TIMER peripheral
void nvic_config(void);   // configure the TIMER1 interrupt

void sensor_generateOctlBuff(void);
void sensor_dmaOctlConfig(void);
void sensor_adcConfig(void);

void sensor_signalProcessing(void);
void generateLuts(void);

void i2c_config(void);
void i2c_busScan(void);

void ssd1306_DisplayAngle(void);

/* ============================ functions ============================= */
int main(void)
{
    /* configure systick */
    systick_config();

    /* configure Serial Wire Debug (SWD) */
    swd_config(); //Disbale JTAG to Free PA15 used for period output IO

    /* Generate sensor output signals template: port output control register lookup table */
    sensor_generateOctlBuff();

    generateLuts(); // Generate sine and cosine angle lookup tables

    /* configure LED's */
    leds_config();

    /* configure sensor signal pins */
    sensor_config();

    /* Timer config, sensor signals timebase */
    timer_config();

    /* configure UART printf */
    uart_config();

    /* configure DMA to output OCTL signals */
    sensor_dmaOctlConfig();

    sensor_adcConfig();

    i2c_config();

    ssd1306_Init();

    /* configure interrupts */
    nvic_config();

    led_on(1);

    while (1)
    {
        // led_toggle(1);
        // led_toggle(2);
        sensor_signalProcessing();
        printf("%.2f,%.2f\n", phaseDeg, magnitude);
        ssd1306_DisplayAngle();

        // i2c_busScan();
        //delay_1ms(10);
    }
}

void leds_config()
{
    /* enable the led clock */
    rcu_periph_clock_enable(LD1_GPIO_CLK);
    rcu_periph_clock_enable(LD2_GPIO_CLK);

    /* configure led GPIO port */
    gpio_init(LD1_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LD1_PIN);
    gpio_init(LD2_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LD2_PIN);

    /* turn off leds by default */
    gpio_bit_set(LD1_GPIO_PORT, LD1_PIN);
    gpio_bit_set(LD2_GPIO_PORT, LD2_PIN);
}

void led_on(uint8_t ledNbr)
{
    switch (ledNbr)
    {
    case 1:
        gpio_bit_reset(LD1_GPIO_PORT, LD1_PIN);
        break;
    case 2:
        gpio_bit_reset(LD2_GPIO_PORT, LD2_PIN);
        break;
    default:
        break;
    }
}

void led_off(uint8_t ledNbr)
{
    switch (ledNbr)
    {
    case 1:
        gpio_bit_set(LD1_GPIO_PORT, LD1_PIN);
        break;
    case 2:
        gpio_bit_set(LD2_GPIO_PORT, LD2_PIN);
        break;
    default:
        break;
    }
}

void led_toggle(uint8_t ledNbr)
{
    switch (ledNbr)
    {
    case 1:
        gpio_toggle(LD1_GPIO_PORT, LD1_PIN);
        break;
    case 2:
        gpio_toggle(LD2_GPIO_PORT, LD2_PIN);
        break;
    default:
        break;
    }
}

void sensor_config()
{
    /* Configure sensor output pins */
    rcu_periph_clock_enable(RCU_GPIOA); // The sensor output signals are all on the same port A by design

    gpio_init(SPWM1_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM1_PIN);
    gpio_init(SPWM2_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM2_PIN);
    gpio_init(SPWM3_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM3_PIN);
    gpio_init(SPWM4_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM4_PIN);
    gpio_init(SPWM5_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM5_PIN);
    gpio_init(SPWM6_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM6_PIN);
    gpio_init(SPWM7_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM7_PIN);
    gpio_init(SPWM8_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPWM8_PIN);

    gpio_init(BIAS_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BIAS_PIN);
    gpio_init(ADC_TRIG_OUT_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, ADC_TRIG_OUT_PIN);

    gpio_init(TIMEBASE_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, TIMEBASE_PIN);
    gpio_init(PERIOD_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PERIOD_PIN);

    /* set Input pin configuration */
    rcu_periph_clock_enable(RCU_GPIOB);                                       // The sensor input signals are on the same port
    gpio_init(GND1_GPIO_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GND1_PIN); // ADC input guard (pin next to ADC in, 2 layer pcb no grnd via close to input
    gpio_init(GND2_GPIO_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GND1_PIN); // ADC input guard

    gpio_init(SIGNAL_GPIO_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, SIGNAL_PIN); // Signal input analog in

    /* ADC external trigger config */
    gpio_init(ADC_TRIG_IN_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, ADC_TRIG_IN_PIN); // ADC trigger input
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_11);                         // connect EXTI line to GPIO pin for regular group
    exti_init(EXTI_11, EXTI_EVENT, EXTI_TRIG_RISING);                                            // configure EXTI line for regular group
}

void uart_config()
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(UART1_RX_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(UART1_TX_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, UART1_TX_PIN);

    /* connect port to USARTx_Rx */
    gpio_init(UART1_RX_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, UART1_RX_PIN);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
}

void timer_config(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);

    timer_struct_para_init(&timer_initpara); // initialize TIMER init struct

    /* TIMER1 configuration */
    timer_initpara.prescaler = 460; // 460 Good result with amplifier gain of 11X; // 200 same timebase as caliper ASIC
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 1;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* TIMER1 update DMA request enable */
    timer_dma_enable(TIMER1, TIMER_DMA_UPD);

    timer_enable(TIMER1);
}

/**
 * @brief Calculate buffer index given an offset and buffersize
 * @warning No check for offset > buffSize in place
 * 
 * @param startPos 
 * @param offSet 
 * @param buffSize 
 * @return uint32_t new buffer index
 */
inline uint32_t calcBuffLoc(uint32_t startPos, uint32_t offSet, uint32_t buffSize)
{
    uint32_t newLoc = startPos + offSet;

    if (newLoc < buffSize)
    {
        return newLoc;
    }
    else
    {
        return newLoc - buffSize; // Loop Around
    }
}

/**
 * @brief Configure SWD as only Debug interface, because the default JTAG + SWD conflicts with PA15 (JTAG)
 */
void swd_config()
{
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
}

/**
 * @brief Retarget the C library printf function to the USART
 * 
 * @param ch Ascii character
 * @return int 
 */
int fputc(int ch, __attribute__((unused)) FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
        ;
    return ch;
}

/**
 * @brief Toggle
 * 
 * @param gpio_periph 
 * @param pin 
 */
void gpio_toggle(uint32_t gpio_periph, uint32_t pin)
{
    (gpio_output_bit_get(gpio_periph, pin)) ? gpio_bit_reset(gpio_periph, pin) : gpio_bit_set(gpio_periph, pin);
}

void sensor_generateOctlBuff()
{
    /* Sensor signal templates */
    const uint8_t txBuff[TX_BUFF_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
    const uint8_t biasBuff[BIAS_BUFF_SIZE] = {0, 0, 0, 1, 1, 1, 1, 1};

    /* loop variables */
    uint32_t tx1Loc, tx2Loc, tx3Loc, tx4Loc, tx5Loc, tx6Loc, tx7Loc, tx8Loc = 0; // 0 ... 512-1: TX buffer locations
    uint32_t biasLoc = 0;                                                        // 0 ... 8-1: Bias buffer location
    uint32_t period = 0;                                                         // Period signal current output pin state
    uint32_t timeBase = 0;                                                       // Timebase signal current output pin state
    uint32_t adcTrig = 0;                                                        // ADC external trigger signal current output pin state
    uint32_t octlVal = 0;                                                        // Output control register state for all signals combined

    for (uint32_t i = 0; i < TX_BUFF_SIZE; i++)
    {
        tx1Loc = i;
        /* Calculate other TX signals LUT location 45 degrees phase shifted */
        tx2Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 1, TX_BUFF_SIZE);
        tx3Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 2, TX_BUFF_SIZE);
        tx4Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 3, TX_BUFF_SIZE);
        tx5Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 4, TX_BUFF_SIZE);
        tx6Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 5, TX_BUFF_SIZE);
        tx7Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 6, TX_BUFF_SIZE);
        tx8Loc = calcBuffLoc(tx1Loc, TX_BUFF_SIZE / 8 * 7, TX_BUFF_SIZE);

        /* calculate debug signals */
        period = (tx1Loc < TX_BUFF_SIZE / 2) ? 0 : 1;
        timeBase = (tx1Loc % 2) ? 0 : 1; //flips every sample, timebase for debugging

        /* Calculate ADC external trigger signal */
        adcTrig = ((tx1Loc + 7) % 8) ? 0 : 1; //ADC trigger signal every 8T, Offset by 6 to align ADC trigger (rising edge) on second period when the bias is low.

        /* building the OCTL value */
        octlVal = 0; //Set all pins low default

        if (txBuff[tx1Loc])
            octlVal |= SPWM1_PIN;
        if (txBuff[tx2Loc])
            octlVal |= SPWM2_PIN;
        if (txBuff[tx3Loc])
            octlVal |= SPWM3_PIN;
        if (txBuff[tx4Loc])
            octlVal |= SPWM4_PIN;
        if (txBuff[tx5Loc])
            octlVal |= SPWM5_PIN;
        if (txBuff[tx6Loc])
            octlVal |= SPWM6_PIN;
        if (txBuff[tx7Loc])
            octlVal |= SPWM7_PIN;
        if (txBuff[tx8Loc])
            octlVal |= SPWM8_PIN;
        if (biasBuff[biasLoc])
            octlVal |= BIAS_PIN; // Bias signal: 8T Period 3T LOW 5T HIGH
        if (adcTrig)
            octlVal |= ADC_TRIG_OUT_PIN; // ADC external trigger signal: 8T Period
        if (1)                           // TODO REMOVE! change back to (timeBase),
            octlVal |= TIMEBASE_PIN;     // DBG signal: TimeBase smalles period 1T
        if (period)
            octlVal |= PERIOD_PIN; // DBG signal: Period is the modulated Sine period. 512/2=256, 256T HIGH followed by 256T LOW

        /* Store OCTL value in buffer */
        octlBuff[i] = octlVal;

        /* Loop over bias signal template buffer when end is reached*/
        biasLoc++;
        if (biasLoc >= BIAS_BUFF_SIZE)
            biasLoc = 0;
    }
}

void sensor_dmaOctlConfig()
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);

    /* initialize DMA channel */
    dma_deinit(DMA0, DMA_CH1);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)octlBuff;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_init_struct.number = TX_BUFF_SIZE;
    dma_init_struct.periph_addr = (uint32_t)(&GPIO_OCTL(GPIOA));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH1, &dma_init_struct);

    /* configure DMA mode */
    dma_circulation_enable(DMA0, DMA_CH1);
    dma_memory_to_memory_disable(DMA0, DMA_CH1);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH1);
}

void sensor_adcConfig()
{
    ///clk
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);

    ///dma
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(&samples);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 64;
    dma_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    /* enable DMA circulation mode */
    dma_circulation_enable(DMA0, DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);

    ///ADC
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE); /*!< all the ADCs work independently */
    /* ADC continous function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_9, ADC_SAMPLETIME_1POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_EXTI_11);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

void sensor_signalProcessing()
{
    /*Split even and odd samples */
    int eLoc = 0;
    int oLoc = 0;
    for (uint32_t i = 0; i < 64; i++)
    {
        if (i % 2)
        {
            even[eLoc] = (double)samples[i];
            eLoc++;
        }
        else
        {
            odd[oLoc] = (double)samples[i];
            oLoc++;
        }
    }

    /* demodulate: convert sampled signal to sine*/
    for (uint32_t i = 0; i < 32; i++)
    {
        demol[i] = even[i] - odd[i];
    }

    //Demol: LP filter test 4 sample avg
    for (uint32_t i = 0; i < 32; i++)
    {
        demolLp[i] = (demol[i] + demol[calcBuffLoc(i, 1, 32)] + demol[calcBuffLoc(i, 2, 32)] + demol[calcBuffLoc(i, 3, 32)]) / 4;
    }

    //Demol: LP filter test 8 sample avg
    // for (int i = 0; i < 32; i++)
    // {
    //     demolLp[i] = (demol[i] + demol[calcBuffLoc(i, 1, 32)] + demol[calcBuffLoc(i, 2, 32)] + demol[calcBuffLoc(i, 3, 32)] + demol[calcBuffLoc(i, 4, 32)] + demol[calcBuffLoc(i, 5, 32)] + demol[calcBuffLoc(i, 6, 32)] + demol[calcBuffLoc(i, 7, 32)]) / 8;
    // }

    /// calculate DFT for single frequency bin @ 1Hz
    /// The sampling frequency in Hz = 64 hz
    ///
    /// Bin frequency = k*SamplerateHz/SampleSize = 1*64/64 = 1Hz
    // double sumrealTemp = 0;
    // double sumimagTemp = 0;
    // for (int i = 0; i < 32; i++)
    // {
    //     double angle = (2 * M_PI * (double)i * k) / 32;

    //     // sumrealTemp += (double)demol[i] * cos(angle);
    //     // sumimagTemp += -(double)demol[i] * sin(angle);

    //     sumrealTemp += (double)demolLp[i] * cos(angle);
    //     sumimagTemp += -(double)demolLp[i] * sin(angle);
    // }

    /* Fast dft using precomputed lookup tables for cos and sine part*/
    double sumrealTemp = 0;
    double sumimagTemp = 0;

    for (uint32_t i = 0; i < 32; i++)
    {
        sumrealTemp += (double)demolLp[i] * cosAngLut[i];
        sumimagTemp += -(double)demolLp[i] * sinAngLut[i];
    }

    // dft coefficients at the frequency bin of interest (1hz) in complex form
    real = sumrealTemp; //X-axis
    imag = sumimagTemp; //Y-axis

    // Calculate phase
    if (real >= 0 && imag >= 0)
    {
        //1-Quadrant (top right) [+, +]
        phaseRad = atan2(imag, real);
        quadrant = 1;
    }
    else if (real < 0 && imag >= 0)
    {
        //2-Quadrant (top left) [-, +]
        phaseRad = atan2(imag, real);
        quadrant = 2;
    }
    else if (real < 0 && imag < 0)
    {
        //3-Quadrant (bottom left) [-, -]
        phaseRad = 2 * M_PI + atan2(imag, real);
        quadrant = 3;
    }
    else if (real >= 0 && imag < 0)
    {
        //4-Quadrant (bottom right) [+, -] #Optimization: do not check last quadrant just use else
        phaseRad = 2 * M_PI + atan2(imag, real);
        quadrant = 4;
    }

    // phaseDeg = (phaseRad * 180) / M_PI; // 0 to 360 degrees
    phaseDeg = round((phaseRad * 180) / M_PI);

    // Calculate magnitude
    magnitude = sqrt(fabs(real) + fabs(imag));
}

void generateLuts()
{
    for (int i = 0; i < 32; i++)
    {
        double angle = (2 * M_PI * (double)i * k) / 32;

        cosAngLut[i] = cos(angle);
        sinAngLut[i] = sin(angle);
    }
}

void ssd1306_delay(uint32_t delayMs)
{
    delay_1ms(delayMs);
}

//void eeprom_page_write(uint8_t* p_buffer, uint8_t write_address, uint8_t number_of_byte)
void ssd1306_i2c_mem_write(uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
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
    i2c_master_addressing(I2C0, (uint32_t)DevAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
        ;

    /* clear the ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

    /* wait until the transmit data buffer is empty */
    while (SET != i2c_flag_get(I2C0, I2C_FLAG_TBE))
        ;

    /* send the EEPROM's internal address to write to : only one byte address */
    i2c_data_transmit(I2C0, MemAddress);

    /* wait until BTC bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
        ;

    /* while there is data to be written */
    while (Size--)
    {
        i2c_data_transmit(I2C0, *pData);

        /* point to the next byte to be written */
        pData++;

        /* wait until BTC bit is set */
        while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
            ;
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);

    /* wait until the stop condition is finished */
    while (I2C_CTL0(I2C0) & 0x0200)
        ;
}

void ssd1306_DisplayAngle()
{
    const uint8_t x = (128 - (3 * 16)) / 2;
    const uint8_t y = 6;
    char stringBuff[] = "xxxx";
    sprintf(stringBuff, "%i", (int)phaseDeg);

    ssd1306_Fill(Black);

    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(stringBuff, Font_16x26, White);

    ssd1306_UpdateScreen();
}
