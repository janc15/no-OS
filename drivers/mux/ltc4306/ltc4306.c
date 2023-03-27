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
int ltc4306_set_register_value(struct ltc4306_dev *dev,
				uint8_t register_address,
				uint8_t register_value)
{
	static uint8_t write_data[2] = {0, 0};

	write_data[0] = register_address;
	write_data[1] = register_value;
	
	return no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param dev              - The device structure.
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/

int ltc4306_get_register_value(struct ltc4306_dev *dev,
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

/***************************************************************************//**
 * @brief Free the resources allocated by ltc4306_init().
 *
 * @param dev - The device structure.
 * @param register_address - Address of the register
 * @param connect_to_bus - Bus (can be any value from 1 to 4) the user wants to connect to.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_connect_to_downstream_channel(struct ltc4306_dev *dev,
				   uint8_t register_address, uint8_t connect_to_bus)
{
	uint8_t read_data[2]   = {0, 0};
	uint8_t *register_value;
	uint8_t bus_state_check;

	//reads register address and checks if it is Register 3 (0x03)
	read_data[0] = register_address;

	if (read_data[0] != 0x03)
		return -1;

	//gets register value; AND to 0x0F to only first 4 bits
	no_os_i2c_write(dev->i2c_desc, read_data, 1, 0);
	no_os_i2c_read(dev->i2c_desc, read_data, 1, 1);
	*register_value = read_data;

	bus_state_check = *(register_value + 1);
	bus_state_check &= 0x0F;
	*(register_value + 2) = 0x03;
	
	// first if statement masks bit depending on what bus you want to connect
	if (connect_to_bus == 0x01)
		bus_state_check &= 0x08;
		//checks bus logic state, if high, will set bus state fet high to connect to bus
			if (bus_state_check == 0x08)
				*(register_value + 1) |= 0x20;
				return no_os_i2c_write(dev->i2c_desc, read_data, 2, 1);
	
	else if (connect_to_bus == 0x02)
		bus_state_check &= 0x04;
			if (bus_state_check == 0x04)
				*(register_value + 1) |= 0x40;
				return no_os_i2c_write(dev->i2c_desc, read_data, 2, 1);

	else if (connect_to_bus == 0x03)
		bus_state_check &= 0x02;
			if (bus_state_check == 0x02)
				*(register_value + 1) |= 0x20;
				return no_os_i2c_write(dev->i2c_desc, read_data, 2, 1);

	else if (connect_to_bus == 0x04)
		bus_state_check &= 0x01;
			if (bus_state_check == 0x01)
				*(register_value + 1) |= 0x10;
				return no_os_i2c_write(dev->i2c_desc, read_data, 2, 1);
}

/***************************************************************************//**
 * @brief Free the resources allocated by ltc4306_init().
 *
 * @param dev - The device structure.
 * @param upstream_en - Set true to enable upstream rise time accelerator.
 * @param downstream_en - Set true to enable downstream rise time accelerator.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_upstream_downstream_accelerator_en(struct ltc4306_dev *dev, bool upstream_en, bool downstream_en)
{
	static uint8_t write_data[2] = {0, 0};
	*ptr = write_data;
	*(ptr + 1) = 0x01;

	if (upstream_en == true && downstream_en == true)
		*(ptr + 2) |= 0xC0;
		return no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);

	else if (upstream_en == false && downstream_en == true)
		*(ptr + 2) |= 0x40;
		return no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);

	if (upstream_en == true && downstream_en == false)
		*(ptr + 2) |= 0x80;
		return no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);

	else
		return no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);

}

/***************************************************************************//**
 * @brief Free the resources allocated by ltc4306_init().
 *
 * @param dev - The device structure.
 * @param device_address - Address of slave device that sent interrupt signal.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_alert_response(struct ltc4306_dev *dev, uint8_t device_address)
{
	static uint8_t read_data[2] = {0, 0};
	static uint8_t register_value = 0;

	read_data[0] = 0x19;
	read_data[1] = device_address;
	no_os_i2c_write(dev->i2c_desc, read_data, 1, 0);
	no_os_i2c_read(dev->i2c_desc, read_data, 1, 1);
	register_value = read_data[0];

	return register_value;
}

/***************************************************************************//**
 * @brief Free the resources allocated by ltc4306_init().
 *
 * @param dev - The device structure.
 * @param mass_write_en - Set true to enable mass write address.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_mass_write_en(struct ltc4306_dev *dev, bool mass_write_en)
{
	uint8_t register_value;

	register_value = ltc4306_get_register_value(dev, 0x02);

	register_value |= 0x04;

	return ltc4306_set_register_value(dev, 0xBA, register_value);
}

/***************************************************************************//**
 * @brief Free the resources allocated by ltc4306_init().
 *
 * @param dev - The device structure.
 * @param gpio1_mode_config - Set true to configure GPIO1 to input mode; false for output mode.
 * @param gpio2_mode_config - Set true to configure GPIO2 to input mode; false for output mode.
 * @param gpio1_output_mode_config - Set true to configure GPIO1 output mode to push-pull; false for open-drain pull-down.
 * @param gpio2_output_mode_config - Set true to configure GPIO2 output mode to push-pull; false for open-drain pull-down.
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_gpio_mode_configure(struct ltc4306_dev *dev, bool gpio1_mode_config, bool gpio2_mode_config, 
								bool gpio1_output_mode_config, bool gpio2_output_mode_config)
{
	uint8_t register_address = 0x02;
	uint8_t register_value = 0;

	register_value = ltc4306_get_register_value(dev, register_address);
	
	if (gpio1_mode_config)
	{
		register_value |= 0x80;
		if (gpio1_output_mode_config)
			register_value |= 0x10;
		else
			register_value &= 0xEF;
	}
	else 
		register_value &= 0x6F;
	
	if (gpio2_mode_config)
	{
		register_value |= 0x40;
		if (gpio2_output_mode_config)
			register_value |= 0x08;
		else
			register_value &= 0xF7;
	}
	else 
		register_value &= 0xB7;
	
	return ltc4306_set_register_value(dev, register_address, register_value);
}