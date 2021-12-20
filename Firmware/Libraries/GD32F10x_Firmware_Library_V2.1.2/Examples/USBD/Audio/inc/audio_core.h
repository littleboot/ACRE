/*!
    \file  audio_core.h
    \brief the header file of USB audio device class core functions

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

#ifndef AUDIO_CORE_H
#define AUDIO_CORE_H

#include "usbd_std.h"


#define USB_SPEAKER_CONFIG_DESC_SIZE                 109
#define FORMAT_24BIT(X)  (uint8_t)(X);(uint8_t)(X >> 8);(uint8_t)(X >> 16)

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_OUT_BIT_RESOLUTION                   16
#define DEFAULT_OUT_CHANNEL_NBR                      2 /* Mono = 1, Stereo = 2 */
#define AUDIO_OUT_PACKET                             (uint32_t)(((USBD_AUDIO_FREQ_16K * \
                                                                 (DEFAULT_OUT_BIT_RESOLUTION / 8) *\
                                                                  DEFAULT_OUT_CHANNEL_NBR) / 1000))

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
   that it is an even number and higher than 3 */
#define OUT_PACKET_NUM                               4

/* Total size of the audio transfer buffer */
#define OUT_BUF_MARGIN                               4
#define TOTAL_OUT_BUF_SIZE                           ((uint32_t)((AUDIO_OUT_PACKET + OUT_BUF_MARGIN) * OUT_PACKET_NUM))

#define AUDIO_CONFIG_DESC_SIZE                       109
#define AUDIO_INTERFACE_DESC_SIZE                    9
#define USB_AUDIO_DESC_SIZ                           0x09
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE            0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE           0x07

#define AUDIO_DESCRIPTOR_TYPE                        0x21
#define USB_DEVICE_CLASS_AUDIO                       0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                  0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING                0x02
#define AUDIO_PROTOCOL_UNDEFINED                     0x00
#define AUDIO_STREAMING_GENERAL                      0x01
#define AUDIO_STREAMING_FORMAT_TYPE                  0x02

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE              0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE               0x25

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                         0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                 0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL                0x03
#define AUDIO_CONTROL_FEATURE_UNIT                   0x06

#define AUDIO_INPUT_TERMINAL_DESC_SIZE               0x0C
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE              0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE          0x07

#define AUDIO_CONTROL_MUTE                           0x0001

#define AUDIO_FORMAT_TYPE_I                          0x01
#define AUDIO_FORMAT_TYPE_III                        0x03

#define USB_ENDPOINT_TYPE_ISOCHRONOUS                0x01
#define AUDIO_ENDPOINT_GENERAL                       0x01

#define AUDIO_REQ_GET_CUR                            0x81
#define AUDIO_REQ_SET_CUR                            0x01

#define AUDIO_OUT_STREAMING_CTRL                     0x02

/** @defgroup USBD_AUDIO_Core_Exported_Macros
  * @{
  */
#define PACKET_SIZE(freq)             ((freq * 2) * 2 / 1000)

#define AUDIO_PACKET_SIZE(frq)        (uint8_t)(PACKET_SIZE(frq) & 0xFF), \
                                      (uint8_t)((PACKET_SIZE(frq) >> 8) & 0xFF)

#define SAMPLE_FREQ(frq)              (uint8_t)(frq), (uint8_t)((frq >> 8)), \
                                      (uint8_t)((frq >> 16))


#pragma pack(1)

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint16_t bcdADC;                      /*!<  */
    uint16_t wTotalLength;                /*!<  */
    uint8_t  bInCollection;               /*!<  */
    uint8_t  baInterfaceNr;               /*!<  */
} usb_descriptor_AC_interface_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bTerminalLink;               /*!<  */
    uint8_t  bDelay;                      /*!<  */
    uint16_t wFormatTag;                  /*!<  */
} usb_descriptor_AS_interface_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bTerminalID;                 /*!<  */
    uint16_t wTerminalType;               /*!<  */
    uint8_t  bAssocTerminal;              /*!<  */
    uint8_t  bNrChannels;                 /*!<  */
    uint16_t wChannelConfig;              /*!<  */
    uint8_t  iChannelNames;               /*!<  */
    uint8_t  iTerminal;                   /*!<  */
} usb_descriptor_input_terminal_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bTerminalID;                 /*!<  */
    uint16_t wTerminalType;               /*!<  */
    uint8_t  bAssocTerminal;              /*!<  */
    uint8_t  bSourceID;                   /*!<  */
    uint8_t  iTerminal;                   /*!<  */
} usb_descriptor_output_terminal_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bUnitID;                     /*!<  */
    uint8_t  bSourceID;                   /*!<  */
    uint8_t  bControlSize;                /*!<  */
    uint8_t bmaControls0;                 /*!<  */
    uint8_t bmaControls1;                 /*!<  */
    uint8_t  iFeature;                    /*!<  */
} usb_descriptor_mono_feature_unit_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bUnitID;                     /*!<  */
    uint8_t  bSourceID;                   /*!<  */
    uint8_t  bControlSize;                /*!<  */
    uint16_t bmaControls0;                /*!<  */
    uint16_t bmaControls1;                /*!<  */
    uint16_t bmaControls2;                /*!<  */
    uint8_t  iFeature;                    /*!<  */
} usb_descriptor_stereo_feature_unit_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bFormatType;                 /*!<  */
    uint8_t  bNrChannels;                 /*!<  */
    uint8_t  bSubFrameSize;               /*!<  */
    uint8_t  bBitResolution;              /*!<  */
    uint8_t  bSamFreqType;                /*!<  */
    uint8_t  bSamFreq[3];                 /*!<  */
} usb_descriptor_format_type_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bEndpointAddress;            /*!<  */
    uint8_t  bmAttributes;                /*!<  */
    uint16_t wMaxPacketSize;              /*!<  */
    uint8_t  bInterval;                   /*!<  */
    uint8_t  bRefresh;                    /*!<  */
    uint8_t  bSynchAddress;               /*!<  */
} usb_descriptor_std_endpoint_struct;     /*!<  */

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size */

    uint8_t  bDescriptorSubtype;          /*!<  */
    uint8_t  bmAttributes;                /*!<  */
    uint8_t  bLockDelayUnits;             /*!<  */
    uint16_t wLockDelay;                  /*!<  */
} usb_descriptor_AS_endpoint_struct;      /*!<  */

#pragma pack()


/* USB configuration descriptor struct */
typedef struct
{
    usb_descriptor_configuration_struct        Config;
    usb_descriptor_interface_struct            Speaker_Std_Interface;
    usb_descriptor_AC_interface_struct         Speaker_AC_Interface;
    usb_descriptor_input_terminal_struct       Speaker_IN_Terminal;
    usb_descriptor_mono_feature_unit_struct    Speaker_Feature_Unit;
    usb_descriptor_output_terminal_struct      Speaker_OUT_Terminal;
    usb_descriptor_interface_struct            Speaker_Std_AS_Interface_ZeroBand;
    usb_descriptor_interface_struct            Speaker_Std_AS_Interface_Opera;
    usb_descriptor_AS_interface_struct         Speaker_AS_Interface;
    usb_descriptor_format_type_struct          Speaker_Format_TypeI;
    usb_descriptor_std_endpoint_struct         Speaker_Std_Endpoint;
    usb_descriptor_AS_endpoint_struct          Speaker_AS_Endpoint;
} usb_descriptor_configuration_set_struct;

extern void* const usbd_strings[USB_STRING_COUNT];
extern const usb_descriptor_device_struct device_descriptor;
extern usb_descriptor_configuration_set_struct configuration_descriptor;

extern usbd_status_enum audio_init (void *pudev, uint8_t config_index);
extern usbd_status_enum audio_deinit (void *pudev, uint8_t config_index);
extern usbd_status_enum audio_req_handler (void *pudev, usb_device_req_struct *req);
extern usbd_status_enum  audio_data_handler (void *pudev, usbd_dir_enum rx_tx, uint8_t ep_id);

#endif
