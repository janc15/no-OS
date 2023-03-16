/***************************************************************************//**
 *   @file   ltc4306.c
 *   @brief  Implementation of ltc4306 Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include "ltc4306.h"			// ltc4306 definitions.

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 * @param register_value   - Data value to write.
 *
 * @return None.
*******************************************************************************/
void ltc4306_set_register_value(struct ltc4306_dev *dev,
				uint8_t register_address,
				uint8_t register_value)
{
	static uint8_t write_data[2] = {0, 0};

	write_data[0] = register_address;
	write_data[1] = register_value;
	no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
uint8_t ltc4306_get_register_value(struct ltc4306_dev *dev,
				   uint8_t register_address)
{
	static uint8_t read_data[2]   = {0, 0};
	static uint8_t register_value = 0;

	read_data[0] = register_address;
	no_os_i2c_write(dev->i2c_desc, read_data, 1, 0);
	no_os_i2c_read(dev->i2c_desc, read_data, 1, 1);
	register_value = read_data[0];

	return register_value;
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ltc4306
 *		  part is present.
 *
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: -1 - I2C peripheral was not initialized or
 *                                ltc4306 part is not present.
 *                            0 - I2C peripheral is initialized and ltc4306
 *                                part is present.
*******************************************************************************/
int8_t ltc4306_init(struct ltc4306_dev **device,
		    struct ltc4306_init_param init_param)
{
	struct ltc4306_dev *dev;
	static uint8_t status;

	dev = (struct ltc4306_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

	status = no_os_i2c_init(&dev->i2c_desc, &init_param.i2c_init);

	*device = dev;

	return status;
}

/***************************************************************************//**
 * @brief Free the resources allocated by ltc4306_init().
 *
 * @param dev - The device structure.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ltc4306_remove(struct ltc4306_dev *dev)
{
	int32_t ret;

	ret = no_os_i2c_remove(dev->i2c_desc);

	free(dev);

	return ret;
}


uint8_t ltc4306_identify_device_id(struct ltc4306_dev *dev,
				   uint8_t register_address_adr0,
				   uint8_t register_address_adr1,
				   uint8_t register_address_adr2)
{
	//initiate variables
	static uint8_t device_id = 0;
	static uint8_t register_value_adr0 = 0;
	static uint8_t register_value_adr1 = 0;
	static uint8_t register_value_adr2 = 0;
	
	/**static uint8_t read_data_adr0[2]   = {0, 0};
	static uint8_t read_data_adr1[2]   = {0, 0};
	static uint8_t read_data_adr2[2]   = {0, 0};

	//read register address: ADR0
	read_data_adr0[0] = register_address_adr0;
	no_os_i2c_write(dev->i2c_desc, read_data_adr0, 1, 0);
	no_os_i2c_read(dev->i2c_desc, read_data_adr0, 1, 1);
	register_value_adr0 = read_data_adr0[0];

	//read register address: ADR1
	read_data_adr1[0] = register_address_adr1;
	no_os_i2c_write(dev->i2c_desc, read_data_adr1, 1, 0);
	no_os_i2c_read(dev->i2c_desc, read_data_adr1, 1, 1);
	register_value_adr1 = read_data_adr1[0];

	//read register address: ADR2
	read_data_adr2[0] = register_address_adr2;
	no_os_i2c_write(dev->i2c_desc, read_data_adr2, 1, 0);
	no_os_i2c_read(dev->i2c_desc, read_data_adr2, 1, 1);
	register_value_adr2 = read_data_adr2[0];

/***************************************************************************//**
	 set condition for device_id value
	 H = 2
	 L = 1
	 NC = 0
*******************************************************************************/

	if (register_value_adr2==1 && register_value_adr1==0 && register_value_adr0==1)
		device_id = 0x80;

	else if (register_value_adr2==1 && register_value_adr1==2 && register_value_adr0==0)
		device_id = 0x82;

	else if (register_value_adr2==1 && register_value_adr1==0 && register_value_adr0==0)
		device_id = 0x84;

	else if (register_value_adr2==1 && register_value_adr1==0 && register_value_adr0==2)
		device_id = 0x86;

	else if (register_value_adr2==1 && register_value_adr1==1 && register_value_adr0==1)
		device_id = 0x88;

	else if (register_value_adr2==1 && register_value_adr1==2 && register_value_adr0==2)
		device_id = 0x8A;

	else if (register_value_adr2==1 && register_value_adr1==1 && register_value_adr0==0)
		device_id = 0x8C;

	else if (register_value_adr2==1 && register_value_adr1==1 && register_value_adr0==2)
		device_id = 0x8E;

	else if (register_value_adr2==0 && register_value_adr1==0 && register_value_adr0==1)
		device_id = 0x90;

	else if (register_value_adr2==0 && register_value_adr1==2 && register_value_adr0==0)
		device_id = 0x92;

	else if (register_value_adr2==0 && register_value_adr1==0 && register_value_adr0==0)
		device_id = 0x94;

	else if (register_value_adr2==0 && register_value_adr1==0 && register_value_adr0==2)
		device_id = 0x96;

	else if (register_value_adr2==0 && register_value_adr1==1 && register_value_adr0==1)
		device_id = 0x98;

	else if (register_value_adr2==0 && register_value_adr1==2 && register_value_adr0==2)
		device_id = 0x9A;

	else if (register_value_adr2==0 && register_value_adr1==1 && register_value_adr0==0)
		device_id = 0x9C;

	else if (register_value_adr2==0 && register_value_adr1==1 && register_value_adr0==2)
		device_id = 0x9E;

	else if (register_value_adr2==2 && register_value_adr1==0 && register_value_adr0==1)
		device_id = 0xA0;

	else if (register_value_adr2==2 && register_value_adr1==2 && register_value_adr0==0)
		device_id = 0xA2;

	else if (register_value_adr2==2 && register_value_adr1==0 && register_value_adr0==0)
		device_id = 0xA4;

	else if (register_value_adr2==2 && register_value_adr1==0 && register_value_adr0==2)
		device_id = 0xA6;

	else if (register_value_adr2==2 && register_value_adr1==1 && register_value_adr0==1)
		device_id = 0xA8;
		
	else if (register_value_adr2==2 && register_value_adr1==2 && register_value_adr0==2)
		device_id = 0xAA;

	else if (register_value_adr2==2 && register_value_adr1==1 && register_value_adr0==0)
		device_id = 0xAC;

	else if (register_value_adr2==2 && register_value_adr1==1 && register_value_adr0==2)
		device_id = 0xAE;
	
	else if (register_value_adr2==2 && register_value_adr1==2 && register_value_adr0==1)
		device_id = 0xB0;

	else if (register_value_adr2==1 && register_value_adr1==2 && register_value_adr0==1)
		device_id = 0xB2;

	else if (register_value_adr2==0 && register_value_adr1==2 && register_value_adr0==1)
		device_id = 0xB4;

	return device_id;
}