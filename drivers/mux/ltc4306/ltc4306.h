/***************************************************************************//**
 *   @file   LTC4306.h
 *   @brief  Header file of LTC4306 Driver.
 *   @author Mihai Bancisor (Mihai.Bancisor@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __LTC4306_H__
#define __LTC4306_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"

/******************************************************************************/
/************************** LTC4306 Definitions *******************************/
/******************************************************************************/

#define PMOD_IOXP_J1		0	// J1 port of PmodIOXP
#define PMOD_IOXP_J2		1	// J2 port of PmodIOXP
#define LTC4306_ADDRESS		0x34	// I2C ADDRESS
#define LTC4306_ID		0x10	// Manufacturer ID

/* Register address definitions */
#define LTC4306_CTRL_REG0				0x00
#define LTC4306_CTRL_REG1		       	0x01
#define LTC4306_CTRL_REG2           	0x02
#define LTC4306_CTRL_REG3            	0x03
#define LTC4306_MASS_WRITE            	0xBA
#define LTC4306_ALERT_RESPONSE          0x19
#define LTC4306_0		            	0x80
#define LTC4306_1		            	0x82
#define LTC4306_2		            	0x84
#define LTC4306_3		            	0x86
#define LTC4306_4		            	0x88
#define LTC4306_5		            	0x8A
#define LTC4306_6		            	0x8C
#define LTC4306_7		            	0x8E
#define LTC4306_8		            	0x90
#define LTC4306_9		            	0x92
#define LTC4306_10		            	0x94
#define LTC4306_11		            	0x96
#define LTC4306_12		            	0x98
#define LTC4306_13		            	0x9A
#define LTC4306_14		            	0x9C
#define LTC4306_15		            	0x9E
#define LTC4306_16		            	0xA0
#define LTC4306_17		            	0xA2
#define LTC4306_18		            	0xA4
#define LTC4306_19		            	0xA6
#define LTC4306_20		            	0xA8
#define LTC4306_21		            	0xAA
#define LTC4306_22		            	0xAC
#define LTC4306_23		            	0xAE
#define LTC4306_24		            	0xB0
#define LTC4306_25		            	0xB2
#define LTC4306_26		            	0xB4

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

struct LTC4306_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
};

struct LTC4306_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
};


/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! Writes data into a register. */
void LTC4306_set_register_value(struct LTC4306_dev *dev,
				uint8_t register_address,
				uint8_t register_value);

/*! Reads the value of a register. */
uint8_t LTC4306_get_register_value(struct LTC4306_dev *dev,
				   uint8_t register_address);

/*! Initializes the communication peripheral and checks if the LTC4306
	part is present. */
int8_t LTC4306_init(struct LTC4306_dev **device,
		    struct LTC4306_init_param init_param);

/*! Free the resources allocated by LTC4306_init(). */
int32_t LTC4306_remove(struct LTC4306_dev *dev);

/*! Sets the direction of the pins. */
void LTC4306_gpio_direction(struct LTC4306_dev *dev,
			    uint8_t reg,
			    uint8_t val);

/*! Reads the state of the pins. */
uint8_t LTC4306_get_pin_state(struct LTC4306_dev *dev,
			      uint8_t reg);

/*! Sets the state of the pins.*/
void LTC4306_set_pin_state(struct LTC4306_dev *dev,
			   uint8_t reg,
			   uint8_t state);

/*! Initializes keyboard decoder. */
void LTC4306_init_key(struct LTC4306_dev *dev,
		      uint8_t pmod_port);

#endif	/* __LTC4306_H__ */
