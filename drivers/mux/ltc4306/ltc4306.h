/***************************************************************************//**
 *   @file   LTC4306.h
 *   @brief  Header file of LTC4306 Driver.
 *   @author Janchris Espinoza (Janchris.Espinoza@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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

//#define LTC4306_ADDRESS		0x34	// I2C ADDRESS
//#define LTC4306_ID			0x10	// Manufacturer ID

/* Register address definitions */
#define LTC4306_CTRL_REG0				0x00
#define LTC4306_CTRL_REG1		       	0x01
#define LTC4306_CTRL_REG2           	0x02
#define LTC4306_CTRL_REG3            	0x03
#define LTC4306_MASS_WRITE_ADDR         0xBA
#define LTC4306_ALERT_RESPONSE_ADDR     0x19


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
int ltc4306_write_register_value(struct ltc4306_dev *dev,
				uint8_t register_address,
				uint8_t register_value);

/*! Reads the value of a register. */
int ltc4306_read_register_value(struct ltc4306_dev *dev,
				   uint8_t register_address, uint8_t *register_value);

/*! Initializes the communication peripheral and checks if the LTC4306
	part is present. */
int8_t ltc4306_init(struct ltc4306_dev **device,
		    struct ltc4306_init_param init_param);

/*! Free the resources allocated by LTC4306_init(). */
int32_t ltc4306_remove(struct ltc4306_dev *dev);

/*! Gives the equivalent hex device address based on the input combination
 * 		of ADR0, ADR1, and ADR2. (27 possible device address) */
uint8_t ltc4306_identify_device_id(struct ltc4306_init_param init_param, 
	uint8_t adr0_connection, uint8_t adr1_connection, uint8_t adr2_connection, 
	*device_address);

/*! Connects to a downstream bus. Bus logic state must be 1 for connection
 * 		to occur EXCEPT when conn_req is 1. Controller will connect to 
 * 		downstream bus regardless of its logic state if conn_req is 1. */
int ltc4306_connect_to_downstream_channel(struct ltc4306_dev *dev, 
	uint8_t connect_to_bus);

/*! Sets the state of the pins.*/
void LTC4306_set_pin_state(struct LTC4306_dev *dev,
			   uint8_t reg, uint8_t state);

/*! Enables and disables upstream and downstream accelerator. */
int ltc4306_upstream_downstream_accelerator_en(struct ltc4306_dev *dev, 
	bool upstream_en, bool downstream_en);

/* Enables Mass Write Adddress. */
int ltc4306_mass_write_en(struct ltc4306_dev *dev, bool mass_write_en);

/* Configures GPIO1 and GPIO2 as input or output mode. If selected as
 * 		output mode, it can be configured as open-drain or push-pull. */
int ltc4306_gpio_mode_configure(struct ltc4306_dev *dev, bool gpio1_is_input, 
	bool gpio2_is_input, bool gpio1_is_pushpull, bool gpio2_is_pushpull);

/* Sets Connection Requirement bit. */
int ltc4306_set_connection_requirement(struct ltc4306_dev *dev, 
	bool connect_regardless);

/* Reads bus logic state. */
int ltc4306_read_bus_logic_state(struct ltc4306_dev *dev, uint8_t bus_number, 
	bool *is_high);

/* Reads Downstream Connected bit; indicates if upstream bus is
 * 		connected to any downstream buses. */
int ltc4306_read_downstream_connected_bit(struct ltc4306_dev *dev, 
	bool *is_high);

/* Reads Alert Logic State of ALERT pins. */
int ltc4306_read_alert_logic_state(struct ltc4306_dev *dev, 
	uint8_t alert_pin_number, bool *is_high);

/* Reads Failed Connection Attempt bit. */
int ltc4306_read_failed_connection_attempt_bit(struct ltc4306_dev *dev, 
	bool *is_high);

/* Reads Latched Timeout bit. */
int ltc4306_read_latched_timeout_bit(struct ltc4306_dev *dev, bool *is_high);

/* Sets Timeout Mode bit 0 and bit 1. */
int ltc4306_set_timeout_mode(struct ltc4306_dev *dev, 
	uint8_t timeout_mode_value);

/* Reads GPIO1 and GPIO 2 logic states. */
int ltc4306_read_gpio_logic_state(struct ltc4306_dev *dev, 
	uint8_t *register_value);

/* Sets GPIO Output Driver state. */
int ltc4306_write_gpio_output_state(struct ltc4306_dev *dev, bool gpio1_is_high, 
	bool gpio2_is_high);

#endif	/* __LTC4306_H__ */
