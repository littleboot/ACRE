/*!
    \file  gd32f103c_audio_codec.c
    \brief the low layer driver for audio codec

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

#include "gd32f103c_audio_codec.h"
#include "usbd_conf.h"
#include "gd32f10x_spi.h"

/* mask for the bit en of the i2s cfgr register */
#define I2S_ENABLE_MASK                 (0x0400)

/* delay for the codec to be correctly reset */
#define CODEC_RESET_DELAY               (0x4FFF)

/* codec audio standards */
#ifdef I2S_STANDARD_PHILLIPS
    #define CODEC_STANDARD                 0x04
    #define I2S_STANDARD                   I2S_STD_PHILLIPS
#elif defined(I2S_STANDARD_MSB)
    #define CODEC_STANDARD                 0x00
    #define I2S_STANDARD                   I2S_STD_MSB
#elif defined(I2S_STANDARD_LSB)
    #define CODEC_STANDARD                 0x08
    #define I2S_STANDARD                   I2S_STD_LSB
#else 
    #error "Error: No audio communication standard selected !"
#endif /* I2S_STANDARD */

/* the 7 bits codec adress (sent through i2c interface) */
#define CODEC_ADDRESS                      0x94  /* b00100111 */

/* this structure is declared glabal because it is handled by two different functions */
static dma_parameter_struct dma_initstructure;
static uint8_t output_dev = 0;

uint32_t audio_totalsize = 0xFFFF; /* the total size of the audio file */
uint32_t audio_remsize   = 0xFFFF; /* the remaining data in audio file */
uint16_t *current_pos;             /* the current poisition of audio pointer */
uint32_t i2s_audiofreq = 0;

__IO uint32_t  codec_timeout = CODEC_LONG_TIMEOUT;

/* low layer codec functions */
static void     codec_ctrl_interface_init    (void);
static void     codec_ctrl_interface_deinit  (void);
static void     codec_audio_interface_init   (uint32_t audio_freq);
static void     codec_audio_interface_deinit (void);
static void     codec_reset                  (void);
static uint32_t codec_write_register         (uint32_t register_addr, uint32_t register_value);
static void     codec_gpio_init              (void);
static void     codec_gpio_deinit            (void);

static void     delay                        (__IO uint32_t ncount);

#ifdef VERIFY_WRITTENDATA
static uint32_t codec_read_register          (uint32_t register_addr);
#endif /* VERIFY_WRITTENDATA */

/*!
    \brief      configure the audio peripherals
    \param[in]  output_device: audio output device
      \arg        OUTPUT_DEVICE_SPEAKER
      \arg        OUTPUT_DEVICE_HEADPHONE
      \arg        OUTPUT_DEVICE_BOTH
      \arg        OUTPUT_DEVICE_AUTO
    \param[in]  volume: initial volume level (from 0 (mute) to 100 (max))
    \param[in]  audio_freq: Audio frequency used to paly the audio stream
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_init(uint16_t output_device, uint8_t volume, uint32_t audio_freq)
{
    /* perform low layer codec initialization */
    if (codec_init(output_device, VOLUME_CONVERT(volume), audio_freq) != 0) {
        return 1;
    } else {
        /* i2s data transfer preparation:
           prepare the media to be used for the audio transfer from memory to i2s peripheral */
        audio_mal_init();

        /* return 0 when all operations are OK */
        return 0;
    }
}

/*!
    \brief      de-initializes all the resources used by the codec (those initialized 
                by eval_audio_init() function) EXCEPT the I2C resources since they are 
                used by the IOExpander as well (and eventually other modules)
    \param[in]  none
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_deinit(void)
{
    /* DeInitialize the Media layer */
    audio_mal_deinit();

    /* DeInitialize Codec */
    codec_deinit();

    return 0;
}

/*!
    \brief      starts playing audio stream from a data buffer for a determined size
    \param[in]  pbuffer: pointer to the buffer
    \param[in]  size: number of audio data bytes
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_play(uint16_t* pbuffer, uint32_t size)
{
    /* set the total number of data to be played (count in half-word) */
    audio_totalsize = size / 2;

    /* call the audio codec play function */
    codec_play();

    /* update the media layer and enable it for play */  
    audio_mal_play((uint32_t)pbuffer, (uint32_t)(DMA_MAX(audio_totalsize / 2)));

    /* update the remaining number of data to be played */
    audio_remsize = (size / 2) - DMA_MAX(audio_totalsize);

    /* update the current audio pointer position */
    current_pos = pbuffer + DMA_MAX(audio_totalsize);

    return 0;
}

/*!
    \brief      pauses or resumes the audio file stream. in case of using dma, the dma
                pause feature is used. in all cases the i2s peripheral is disabled
    \warning    when calling EVAL_AUDIO_PauseResume() function for pause, only
                this function should be called for resume (use of EVAL_AUDIO_Play() 
                function for resume could lead to unexpected behaviour).
    \param[in]  cmd: AUDIO_PAUSE (or 0) to pause, AUDIO_RESUME (or any value different
                from 0) to resume
    \param[in]  addr: address from/at which the audio stream should resume/pause
    \param[in]  size: number of data to be configured for next resume
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_pause_resume(uint32_t cmd, uint32_t addr, uint32_t size)
{
    if (cmd != AUDIO_PAUSE) {
        /* call the media layer pause/resume function */
        audio_mal_pause_resume(cmd, addr, size);

        /* call the audio codec pause/resume function */
        if (codec_pause_resume(cmd) != 0) {
            return 1;
        } else {
            return 0;
        }
    } else {
        /* Call the Audio Codec Pause/Resume function */
        if (codec_pause_resume(cmd) != 0) {
            return 1;
        } else {
            /* Call the Media layer pause/resume function */
            audio_mal_pause_resume(cmd, addr, size);

            /* Return 0 if all operations are OK */
            return 0;
        }
    }
}

/*!
    \brief      stops audio playing and power down the audio codec
    \param[in]  option: can be
      \arg        CODEC_PDWN_SW for software power off (by writing registers)
                  then no need to reconfigure the codec after power on
      \arg        CODEC_PDWN_HW completely shut down the codec (physically). Then need 
                  to reconfigure the Codec after power on
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_stop (uint32_t option)
{
    /* call audio codec stop function */
    if (codec_stop(option) != 0) {
        return 1;
    } else {
        /* call media layer stop function */
        audio_mal_stop();

        /* update the remaining data number */
        audio_remsize = audio_totalsize;    

        /* return 0 when all operations are correctly done */
        return 0;
    }
}

/*!
    \brief      controls the current audio volume level
    \param[in]  volume: volume level to be set in percentage from 0% to 100% (0 for 
                mute and 100 for max volume level)
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_volume_ctl(uint8_t volume)
{
    /* Call the codec volume control function with converted volume value */
    return (codec_volume_ctrl(VOLUME_CONVERT(volume)));
}

/*!
    \brief      enable or disable the mute mode by software
    \param[in]  cmd: could be audio_mute_on to mute sound or audio_mute_off to 
                unmute the codec and restore previous volume level
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t eval_audio_mute (uint32_t cmd)
{
    /* Call the Codec Mute function */
    return (codec_mute(cmd));
}

/*!
    \brief      this function handles main media layer interrupt
    \param[in]  none
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
void Audio_MAL_IRQHandler(void)
{
#ifndef AUDIO_MAL_MODE_NORMAL
    uint16_t *addr = (uint16_t *)current_pos;
    uint32_t size = audio_remsize;
#endif /* AUDIO_MAL_MODE_NORMAL */

#ifdef AUDIO_MAL_DMA_IT_TC_EN
    /* Transfer complete interrupt */
    if (dma_flag_get(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_TC) != RESET)
    {
#ifdef AUDIO_MAL_MODE_NORMAL
        /* Check if the end of file has been reached */
        if (audio_remsize > 0) {
            /* Wait the DMA Stream to be effectively disabled */
            while (DMA_GetCmdStatus(AUDIO_MAL_DMA_STREAM) != DISABLE) {}

            /* Clear the Interrupt flag */
            DMA_ClearFlag(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_TC);  

            /* Re-Configure the buffer address and size */
            dma_initstructure.DMA_Memory0BaseAddr = (uint32_t) current_pos;
            dma_initstructure.DMA_BufferSize = (uint32_t) (DMA_MAX(audio_remsize));

            /* Configure the DMA Stream with the new parameters */
            DMA_Init(AUDIO_MAL_DMA_STREAM, &dma_initstructure);

            /* Enable the I2S DMA Stream*/
            DMA_Cmd(AUDIO_MAL_DMA_STREAM, ENABLE);

            /* Update the current pointer position */
            current_pos += DMA_MAX(audio_remsize);

            /* Update the remaining number of data to be played */
            audio_remsize -= DMA_MAX(audio_remsize);
        } else {
            /* Disable the I2S DMA Stream*/
            DMA_Cmd(AUDIO_MAL_DMA_STREAM, DISABLE);   

            /* Clear the Interrupt flag */
            DMA_ClearFlag(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_TC);       

            /* Manage the remaining file size and new address offset: This function 
            should be coded by user (its prototype is already declared in gd32f150r_audio_codec.h) */  
            EVAL_AUDIO_TransferComplete_CallBack((uint32_t)current_pos, 0);
        }

#elif defined(AUDIO_MAL_MODE_CIRCULAR)
        /* Manage the remaining file size and new address offset: This function 
           should be coded by user (its prototype is already declared in gd32f150r_audio_codec.h) */  
        eval_audio_transfercomplete_callback(pAddr, Size);    

        /* Clear the Interrupt flag */
        DMA_ClearFlag(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_TC);
#endif /* AUDIO_MAL_MODE_NORMAL */  
    }
#endif /* AUDIO_MAL_DMA_IT_TC_EN */

#ifdef AUDIO_MAL_DMA_IT_HT_EN
    /* half transfer complete interrupt */
    if (DMA_GetFlagStatus(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_HT) != RESET)
    {
        /* Manage the remaining file size and new address offset: This function 
           should be coded by user (its prototype is already declared in gd32f150r_audio_codec.h) */  
        eval_audio_halftransfer_callback((uint32_t)addr, size);

        /* Clear the Interrupt flag */
        DMA_ClearFlag(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_HT);
    }
#endif /* AUDIO_MAL_DMA_IT_HT_EN */
  
#ifdef AUDIO_MAL_DMA_IT_TE_EN  
    /* FIFO Error interrupt */
    if ((DMA_GetFlagStatus(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_TE) != RESET) || \
        (DMA_GetFlagStatus(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_FE) != RESET) || \
        (DMA_GetFlagStatus(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_DME) != RESET)) {
       /* manage the error generated on DMA FIFO: This function 
          should be coded by user (its prototype is already declared in gd32f150r_audio_codec.h) */  
        eval_audio_error_callback((uint32_t*)&addr);

        /* clear the interrupt flag */
        DMA_ClearFlag(AUDIO_MAL_DMA_STREAM, AUDIO_MAL_DMA_FLAG_TE | AUDIO_MAL_DMA_FLAG_FE | \
                      AUDIO_MAL_DMA_FLAG_DME);
    }
#endif /* AUDIO_MAL_DMA_IT_TE_EN */
}

/* PCM1770 Audio Codec Control Functions */

/*!
    \brief      initializes the audio codec and all related interfaces (control 
                interface: i2c and audio interface: i2s)
    \param[in]  output_device:
      \arg        OUTPUT_DEVICE_SPEAKER
      \arg        OUTPUT_DEVICE_HEADPHONE
      \arg        OUTPUT_DEVICE_BOTH
      \arg        OUTPUT_DEVICE_AUTO
    \param[in]  volume: initial volume level (from 0 (mute) to 100 (max))
    \param[in]  audiofreq: audio frequency used to paly the audio stream
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_init(uint16_t output_device, uint8_t volume, uint32_t audio_freq)
{
    uint32_t counter = 0;

    /* configure the codec related ios */
    codec_gpio_init();

    /* reset the codec registers */
    codec_reset();

    /* initialize the control interface of the audio codec */
    codec_ctrl_interface_init();

    /* keep codec powered off */
    counter += codec_write_register(0x02, 0x01);

    switch (output_device) {
    case OUTPUT_DEVICE_SPEAKER:
        counter += codec_write_register(0x04, 0xFA); /* SPK always ON & HP always OFF */
        output_dev = 0xFA;
        break;
    case OUTPUT_DEVICE_HEADPHONE:
        counter += codec_write_register(0x04, 0xAF); /* SPK always OFF & HP always ON */
        output_dev = 0xAF;
        break;
    case OUTPUT_DEVICE_BOTH:
        counter += codec_write_register(0x04, 0xAA); /* SPK always ON & HP always ON */
        output_dev = 0xAA;
        break;
    case OUTPUT_DEVICE_AUTO:
        counter += codec_write_register(0x04, 0x05); /* detect the HP or the SPK automatically */
        output_dev = 0x05;
        break;
    default:
        counter += codec_write_register(0x04, 0x05); /* detect the HP or the SPK automatically */
        output_dev = 0x05;
        break;
    }

    /* clock configuration: auto detection */
    counter += codec_write_register(0x05, 0x81);

    /* set the slave mode and the audio standard */
    counter += codec_write_register(0x06, CODEC_STANDARD);

    /* set the master volume */
    codec_volume_ctrl(volume);

    /* if the Speaker is enabled, set the Mono mode and volume attenuation level */
    if (output_device != OUTPUT_DEVICE_HEADPHONE) {
        /* Set the Speaker Mono mode */
        counter += codec_write_register(0x0F , 0x06);

        /* Set the Speaker attenuation level */
        counter += codec_write_register(0x24, 0x00);
        counter += codec_write_register(0x25, 0x00);
    }

    /* power on the Codec */
    counter += codec_write_register(0x02, 0x9E);

    /* Additional configuration for the CODEC. These configurations are done to reduce
       the time needed for the Codec to power off. If these configurations are removed,
       then a long delay should be added between powering off the Codec and switching
       off the I2S peripheral MCLK clock (which is the operating clock for Codec).
       If this delay is not inserted, then the codec will not shut down propoerly and
       it results in high noise after shut down. */

    /* disable the analog soft ramp */
    counter += codec_write_register(0x0A, 0x00);

    /* disable the digital soft ramp */
    counter += codec_write_register(0x0E, 0x04);

    /* disable the limiter attack level */
    counter += codec_write_register(0x27, 0x00);

    /* adjust bass and treble levels */
    counter += codec_write_register(0x1F, 0x0F);

    /* adjust pcm volume level */
    counter += codec_write_register(0x1A, 0x0A);
    counter += codec_write_register(0x1B, 0x0A);

    /* configure the i2s peripheral */
    codec_audio_interface_init(audio_freq);

    /* return communication control value */
    return counter;
}

/*!
    \brief      restore the audio codec state to default state and free all used resources
    \param[in]  none
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_deinit(void)
{
    uint32_t counter = 0;

    /* reset the codec registers */
    codec_reset();

    /* keep codec powered off */
    counter += codec_write_register(0x02, 0x01);

    /* deinitialize all use gpios */
    codec_gpio_deinit();

    /* disable the codec control interface */
    codec_ctrl_interface_deinit();

    /* deinitialize the codec audio interface (i2s) */
    codec_audio_interface_deinit();

    /* return communication control value */
    return counter;
}

/*!
    \brief      start the audio codec play feature
                for this codec no play options are required
    \param[in]  none
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_play(void)
{
    /* no actions required on codec level for play command */

    /* return communication control value */
    return 0;
}

/*!
    \brief      pauses and resumes playing on the audio codec
    \param[in]  cmd: AUDIO_PAUSE (or 0) to pause, AUDIO_RESUME (or any value different
                from 0) to resume
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_pause_resume(uint32_t cmd)
{
    uint32_t counter = 0;

    /* pause the audio file playing */
    if (cmd == AUDIO_PAUSE) {
        /* mute the output first */
        counter += codec_mute(AUDIO_MUTE_ON);

        /* put the codec in power save mode */
        counter += codec_write_register(0x02, 0x01);
    } else {
        /* unmute the output first */
        counter += codec_mute(AUDIO_MUTE_OFF);

        counter += codec_write_register(0x04, output_dev);

        /* exit the power save mode */
        counter += codec_write_register(0x02, 0x9E); 
    }

    return counter;
}

/*!
    \brief      stops audio codec playing. it powers down the codec
    \param[in]  codec_pdwn_mode:
      \arg        CODEC_PDWN_SW: only mutes the audio codec. when resuming from this 
                  mode the codec keeps the prvious initialization (no need to re-initialize
                  the codec registers)
      \arg        CODEC_PDWN_HW: physically power down the codec. when resuming from this
                  mode, the codec is set to default configuration (user should re-initialize
                  the codec in order to play again the audio stream)
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_stop(uint32_t codec_pdwn_mode)
{
    uint32_t counter = 0;

    /* mute the output first */
    codec_mute(AUDIO_MUTE_ON);

    if (codec_pdwn_mode == CODEC_PDWN_SW) {
        /* power down the dac and the speaker (pmdac and pmspk bits)*/
        counter += codec_write_register(0x02, 0x9F);
    } else {
        /* power down the dac components */
        counter += codec_write_register(0x02, 0x9F);

        /* wait at least 100ms */
        delay(0xFFF);
    }

    return counter;
}

/*!
    \brief      highers or lowers the codec volume level
    \param[in]  volume: a byte value from 0 to 255 (refer to codec registers 
                description for more details)
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_volume_ctrl(uint8_t volume)
{
    uint32_t counter = 0;

    if (volume > 0xE6) {
        /* set the master volume */
        counter += codec_write_register(0x20, volume - 0xE7);
        counter += codec_write_register(0x21, volume - 0xE7);
    } else {
        /* set the master volume */
        counter += codec_write_register(0x20, volume + 0x19);
        counter += codec_write_register(0x21, volume + 0x19);
    }

    return counter;
}

/*!
    \brief      enables or disables the mute feature on the audio codec
    \param[in]  cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
                mute mode
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_mute(uint32_t cmd)
{
    uint32_t counter = 0;

    /* set the mute mode */
    if (cmd == AUDIO_MUTE_ON)
    {
        counter += codec_write_register(0x04, 0xFF);
    } else {
        counter += codec_write_register(0x04, output_dev);
    }

    return counter;
}

/*!
    \brief      resets the audio codec. it restores the default configuration of the 
                codec (this function shall be called before initializing the codec)
    \note       this function calls an external driver function: The IO Expander driver
    \param[in]  none
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
static void codec_reset(void)
{
    /* no operation */
}

/*!
    \brief      switch dynamically (while audio file is played) the output target (speaker or headphone)
    \note       this function modifies a global variable of the audio codec driver: output_dev.
    \param[in]  none
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
uint32_t codec_switch_output(uint8_t output)
{
    uint8_t counter = 0;

    switch (output) {
    case OUTPUT_DEVICE_SPEAKER:
        counter += codec_write_register(0x04, 0xFA); /* SPK always ON & HP always OFF */
        output_dev = 0xFA;
        break;
    case OUTPUT_DEVICE_HEADPHONE:
        counter += codec_write_register(0x04, 0xAF); /* SPK always OFF & HP always ON */
        output_dev = 0xAF;
        break;
    case OUTPUT_DEVICE_BOTH:
        counter += codec_write_register(0x04, 0xAA); /* SPK always ON & HP always ON */
        output_dev = 0xAA;
        break;
    case OUTPUT_DEVICE_AUTO:
        counter += codec_write_register(0x04, 0x05); /* detect the HP or the SPK automatically */
        output_dev = 0x05;
        break;
    default:
        counter += codec_write_register(0x04, 0x05); /* detect the HP or the SPK automatically */
        output_dev = 0x05;
        break;
    }

    return counter;
}

/*!
    \brief      writes a byte to a given register into the audio codec through the control interface
    \param[in]  register_addr: the address (location) of the register to be written
    \param[in]  register_value: the byte value to be written into destination register
    \param[out] none
    \retval     0 if correct communication, else wrong communication
*/
static uint32_t codec_write_register(uint32_t RegisterAddr, uint32_t RegisterValue)
{
    uint32_t result = 0;

    /* return the verifying value: 0 (passed) or 1 (failed) */
    return result;
}

#ifdef VERIFY_WRITTENDATA

/*!
    \brief      reads and returns te value of an audio codec register through the 
                control interface (I2C)
    \param[in]  register_addr: address of the register to be read
    \param[out] none
    \retval     value of the register to be read or dummy value if the communication fails
*/
static uint32_t codec_read_register(uint32_t RegisterAddr)
{
    uint32_t result = 0;

    /* return the byte read from codec */
    return result;
}

#endif /* VERIFY_WRITTENDATA */

/*!
    \brief      initializes the Audio Codec control interface (I2C)
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void codec_ctrl_interface_init(void)
{
    /* no operation */
}

/*!
    \brief      restore the audio codec control interface to its default state.
                this function doesn't de-initialize the i2c because the i2c peripheral
                may be used by other modules
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void codec_ctrl_interface_deinit(void)
{
    /* no operation */
}

/*!
    \brief      initializes the audio codec audio interface (i2s)
    \note       this function assumes that the i2s input clock (through pll_r in 
                devices reva/z and through dedicated plli2s_r in devices revb/y)
                is already configured and ready to be used
    \param[in]  audio_freq: audio frequency to be configured for the i2s peripheral
    \param[out] none
    \retval     none
*/
static void codec_audio_interface_init(uint32_t audio_freq)
{
    i2s_audiofreq = audio_freq;
    
    /* enable the CODEC_I2S peripheral clock */
    rcu_periph_clock_enable(CODEC_I2S_CLK);

    /* CODEC_I2S peripheral configuration */
    spi_i2s_deinit(CODEC_I2S);

    /* initialize the I2S peripheral with the structure above */
    i2s_psc_config(CODEC_I2S, audio_freq, I2S_FRAMEFORMAT_DT16B_CH16B, 
#ifdef CODEC_MCLK_ENABLED
    I2S_MCKOUT_ENABLE
#elif defined(CODEC_MCLK_DISABLED)
    I2S_MCKOUT_DISABLE
#endif
    );

    i2s_init(CODEC_I2S, I2S_MODE_MASTERTX, I2S_STD_MSB, I2S_CKPL_HIGH);

    /* enable the I2S DMA TX request */
    spi_dma_enable( CODEC_I2S, SPI_DMA_TRANSMIT);
}

/*!
    \brief      restores the audio codec audio interface to its default state
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void codec_audio_interface_deinit(void)
{
    /* disable the codec_i2s peripheral (in case it hasn't already been disabled) */
    i2s_enable(CODEC_I2S);

    /* deinitialize the codec_i2s peripheral */
    spi_i2s_deinit(CODEC_I2S);

    /* disable the codec_i2s peripheral clock */
    rcu_periph_clock_enable(CODEC_I2S_CLK);
}

/*!
    \brief      initializes IOs used by the audio codec (on the control and audio interfaces)
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void codec_gpio_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(CODEC_I2S_GPIO_CLK);
    rcu_periph_clock_enable(CODEC_I2S_MCK_CLK);

    /* CODEC_I2S pins configuraiton: WS, SCK and SD pins */
    gpio_init(CODEC_I2S_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, CODEC_I2S_WS_PIN);
    gpio_init(CODEC_I2S_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, CODEC_I2S_SCK_PIN);
    gpio_init(CODEC_I2S_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, CODEC_I2S_SD_PIN);

#ifdef CODEC_MCLK_ENABLED
    /* codec_i2s pins configuraiton: mck pin */
    gpio_init(CODEC_I2S_MCK_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, CODEC_I2S_MCK_PIN);
#endif /* CODEC_MCLK_ENABLED */
}

/*!
    \brief      restores the ios used by the audio codec interface to their default state
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void codec_gpio_deinit(void)
{
    /* deinitialize all the GPIOs used by the driver (EXCEPT the I2C IOs since 
       they are used by the IOExpander as well) */
    gpio_init(CODEC_I2S_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, CODEC_I2S_WS_PIN | CODEC_I2S_SCK_PIN | CODEC_I2S_SD_PIN);

#ifdef CODEC_MCLK_ENABLED
    /* CODEC_I2S pins deinitialization: MCK pin */
    gpio_init(CODEC_I2S_MCK_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, CODEC_I2S_MCK_PIN);
#endif /* CODEC_MCLK_ENABLED */    
}

/*!
    \brief      inserts a delay time (not accurate timing)
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void delay( __IO uint32_t nCount)
{
    for (; nCount != 0; nCount--);
}

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK

/*!
    \brief      basic management of the timeout situation
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint32_t Codec_TIMEOUT_UserCallback(void)
{
    /* Block communication and all processes */
    while (1)
    {
    }
}

#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

/* Audio MAL Interface Control Functions */

/*!
    \brief      initializes and prepares the Media to perform audio data transfer 
                from Media to the I2S peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void audio_mal_init(void)  
{
#if defined(AUDIO_MAL_DMA_IT_TC_EN) || defined(AUDIO_MAL_DMA_IT_HT_EN) || defined(AUDIO_MAL_DMA_IT_TE_EN)
    NVIC_InitTypeDef NVIC_InitStructure;
#endif

    /* enable the DMA clock */
    rcu_periph_clock_enable(AUDIO_MAL_DMA_CLOCK);

    /* configure the DMA Stream */
    dma_channel_enable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);
    dma_deinit(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

    /* Set the parameters to be configured */
    dma_initstructure.periph_addr = CODEC_I2S_ADDRESS;
    dma_initstructure.memory_addr = (uint32_t)0;/* this field will be configured in play function */
    dma_initstructure.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_initstructure.number = (uint32_t)0xFFFE;/* this field will be configured in play function */
    dma_initstructure.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_initstructure.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_initstructure.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_initstructure.memory_width = DMA_MEMORY_WIDTH_16BIT;

#ifdef AUDIO_MAL_MODE_NORMAL
    dma_circulation_disable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);
#elif defined(AUDIO_MAL_MODE_CIRCULAR)
    dma_circulation_enable(AUDIO_MAL_DMA_CHANNEL);
#else
    #error "AUDIO_MAL_MODE_NORMAL or AUDIO_MAL_MODE_CIRCULAR should be selected !!"
#endif /* AUDIO_MAL_MODE_NORMAL */  

    dma_initstructure.priority = DMA_PRIORITY_HIGH;
    dma_init(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL, &dma_initstructure);

    /* enable the selected dma interrupts (selected in "gd32f150r_audio_codec.h" defines) */
#ifdef AUDIO_MAL_DMA_IT_TC_EN
    dma_interrupt_enable(AUDIO_MAL_DMA_CHANNEL, DMA_INT_FTF);
#endif /* AUDIO_MAL_DMA_IT_TC_EN */

#ifdef AUDIO_MAL_DMA_IT_HT_EN
    DMA_ITConfig(AUDIO_MAL_DMA_CHANNEL, DMA_INT_HTF);
#endif /* AUDIO_MAL_DMA_IT_HT_EN */

#ifdef AUDIO_MAL_DMA_IT_TE_EN
    DMA_ITConfig(AUDIO_MAL_DMA_CHANNEL, DMA_INT_FTF | DMA_INT_HTF | DMA_INT_ERR);
#endif /* AUDIO_MAL_DMA_IT_TE_EN */

   /* enable the I2S DMA request */
   spi_dma_enable(CODEC_I2S, SPI_DMA_TRANSMIT);

#if defined(AUDIO_MAL_DMA_IT_TC_EN) || defined(AUDIO_MAL_DMA_IT_HT_EN) || defined(AUDIO_MAL_DMA_IT_TE_EN)
    /* I2S DMA IRQ channel configuration */
    nvic_irq_enable(AUDIO_MAL_DMA_IRQ, EVAL_AUDIO_IRQ_PREPRIO, EVAL_AUDIO_IRQ_SUBRIO);
#endif 
}

/*!
    \brief      restore default state of the used media
    \param[in]  none
    \param[out] none
    \retval     none
*/
void audio_mal_deinit(void)  
{
#if defined(AUDIO_MAL_DMA_IT_TC_EN) || defined(AUDIO_MAL_DMA_IT_HT_EN) || defined(AUDIO_MAL_DMA_IT_TE_EN)
    NVIC_InitTypeDef NVIC_InitStructure;  

    /* Deinitialize the NVIC interrupt for the I2S DMA Stream */
    NVIC_InitStructure.NVIC_IRQChannel = AUDIO_MAL_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EVAL_AUDIO_IRQ_PREPRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = EVAL_AUDIO_IRQ_SUBRIO;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif 

    /* disable the DMA channel before the deinit */
    dma_channel_disable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

    /* de-initialize the DMA channel */
    dma_deinit(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

    /* The DMA clock is not disabled, since it can be used by other streams */
}

/*!
    \brief      starts playing audio stream from the audio media
    \param[in]  addr: pointer to the audio stream buffer
    \param[in]  size: number of data in the audio stream buffer
    \param[out] none
    \retval     none
*/
void audio_mal_play(uint32_t addr, uint32_t size)
{
    /* disable the I2S DMA Stream*/
    dma_channel_disable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

    /* clear the Interrupt flag */
    dma_interrupt_flag_clear(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL, DMA_INT_FLAG_FTF);

    /* configure the buffer address and size */
    dma_initstructure.memory_addr = (uint32_t)addr;
    dma_initstructure.number = (uint32_t)(size * 2);

    /* configure the DMA Stream with the new parameters */
    dma_init(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL, &dma_initstructure);

    /* enable the I2S DMA Stream*/
    dma_channel_enable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

    /* if the i2s peripheral is still not enabled, enable it */
    if ((SPI_I2SCTL(CODEC_I2S) & I2S_ENABLE_MASK) == 0) {
        i2s_enable(CODEC_I2S);
    }
}

/*!
    \brief      pauses or resumes the audio stream playing from the media
    \param[in]  cmd: AUDIO_PAUSE (or 0) to pause, AUDIO_RESUME (or any value different
                from 0) to resume
    \param[in]  addr: address from/at which the audio stream should resume/pause
    \param[in]  size: number of data to be configured for next resume
    \param[out] none
    \retval     none
*/
void audio_mal_pause_resume(uint32_t cmd, uint32_t addr, uint32_t size)
{
    /* pause the audio file playing */
    if (cmd == AUDIO_PAUSE) {
        /* stop the current DMA request by resetting the I2S cell */
        codec_audio_interface_deinit();

        /* re-configure the I2S interface for the next resume operation */
        codec_audio_interface_init(i2s_audiofreq);

        /* disable the DMA Stream */
        dma_channel_disable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);
            
        /* clear the Interrupt flag */
        dma_flag_clear(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL, AUDIO_MAL_DMA_FLAG_ALL);
    } else {
        /* configure the buffer address and size */
        dma_initstructure.memory_addr = (uint32_t)addr;
        dma_initstructure.number = (uint32_t)(size * 2);

        /* configure the DMA Stream with the new parameters */
        dma_init(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL, &dma_initstructure);

        /* enable the I2S DMA Stream*/
        dma_channel_enable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

        /* if the I2S peripheral is still not enabled, enable it */
        if ((SPI_I2SCTL(CODEC_I2S) & I2S_ENABLE_MASK) == 0) {
            i2s_enable(CODEC_I2S);
        }
    }
}

/*!
    \brief      stops audio stream playing on the used media
    \param[in]  none
    \param[out] none
    \retval     none
*/
void audio_mal_stop(void)
{
    /* stop the Transfer on the I2S side: Stop and disable the DMA stream */
    dma_channel_disable(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL);

    /* clear all the DMA flags for the next transfer */
    dma_flag_clear(AUDIO_MAL_DMA, AUDIO_MAL_DMA_CHANNEL, AUDIO_MAL_DMA_FLAG_ALL);

    /* stop the current DMA request by resetting the I2S cell */
    codec_audio_interface_deinit();

    /* re-configure the I2S interface for the next paly operation */
    codec_audio_interface_init(i2s_audiofreq);
}
