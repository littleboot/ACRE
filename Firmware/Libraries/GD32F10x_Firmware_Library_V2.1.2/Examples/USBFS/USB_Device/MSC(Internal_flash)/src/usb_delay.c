/*!
    \file  usb_delay.c
    \brief USB delay driver

    \version 2014-12-26, V1.0.0, firmware for GD32F10x
    \version 2017-06-20, V2.0.0, firmware for GD32F10x
    \version 2018-07-31, V2.1.0, firmware for GD32F10x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "usb_delay.h"

extern uint8_t timer_prescaler;
__IO uint32_t delay_num = 0;

static void delay_time_set (uint8_t unit);

/*!
    \brief      timer nvic initialization
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer_nvic_init(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(TIMER0_UP_IRQn, 1, 1);
}

/*!
    \brief      timer interrupt handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer_delay_irq (void)
{
    if (RESET != timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
        timer_flag_clear(TIMER0, TIMER_FLAG_UP);

        if (delay_num > 0x00) {
            delay_num--;
        } else {
            timer_disable(TIMER0);
        }
    }
} 

/*!
    \brief      delay routine in microseconds
    \param[in]  time_us: delay Time in microseconds
    \param[out] none
    \retval     none
*/
void delay_us(uint32_t time_us)
{
    delay_num = time_us;
    delay_time_set(TIM_USEC_DELAY);

    while(0U != delay_num);

    timer_disable(TIMER0);
}

/*!
    \brief      delay routine in milliseconds
    \param[in]  time_ms: delay Time in milliseconds
    \param[out] none
    \retval     none
*/
void delay_ms(uint32_t time_ms)
{
    delay_num = time_ms;
    delay_time_set(TIM_MSEC_DELAY);

    while(0U != delay_num);

    timer_disable(TIMER0);
}

/*!
    \brief      set timer
    \param[in]  unit: choose milliseconds or microseconds
    \param[out] none
    \retval     none
*/
static void delay_time_set (uint8_t unit)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_deinit(TIMER0);
  
    if(TIM_USEC_DELAY == unit) {
        timer_initpara.period = 11;
    } else if(TIM_MSEC_DELAY == unit) {
        timer_initpara.period = 11999;
    }
    
    timer_initpara.prescaler         = timer_prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0, &timer_initpara);
    
    timer_update_event_enable(TIMER0);
    timer_interrupt_enable(TIMER0,TIMER_INT_UP);
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    timer_update_source_config(TIMER0, TIMER_UPDATE_SRC_GLOBAL);
  
    /* TIMER0 counter enable */
    timer_enable(TIMER0);
}
