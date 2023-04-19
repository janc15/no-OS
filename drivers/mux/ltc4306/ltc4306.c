/***************************************************************************//**
 *   @file   ltc4306.c
 *   @brief  Implementation of ltc4306 Driver.
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
 * @param dev - The device structure.
 * @param register_address - Address of the register.
 * @param register_value - Data value to write.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_write_register_value(struct ltc4306_dev *dev, uint8_t register_address,
	uint8_t register_value)
{
	uint8_t write_data[2] = {0, 0};

	write_data[0] = register_address;
	write_data[1] = register_value;
	
	return no_os_i2c_write(dev->i2c_desc, write_data, 2, 1);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param dev - The device structure.
 * @param register_address - Address of the register.
 * @param register_value - Stores data read from the register.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/

int ltc4306_read_register_value(struct ltc4306_dev *dev,uint8_t register_address, 
	uint8_t *register_value)
{
	uint8_t read_data[2]   = {0, 0};
	uint8_t ret;

	read_data[0] = register_address;
	ret = no_os_i2c_write(dev->i2c_desc, read_data, 1, 0);

	if (ret)
		return ret;

	*register_value = no_os_i2c_read(dev->i2c_desc, read_data, 1, 1);

	return 0;
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
int8_t ltc4306_init(struct ltc4306_dev **device, struct ltc4306_init_param init_param)
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

/***************************************************************************//**
 * @brief Gives the equivalent hex device address based on the input combination
 * 		of ADR0, ADR1, and ADR2. (27 possible device address)
 *
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 * @param adr0_connection - Used to configure slave address. Input based on 
 * 		ADR0 connection: 0 low logic, 1 high logic, 2 not connected
 * @param adr1_connection - Used to configure slave address. Input based on 
 * 		ADR1 connection: 0 low logic, 1 high logic, 2 not connected
 * @param adr2_connection - Used to configure slave address. Input based on 
 * 		ADR2 connection: 0 low logic, 1 high logic, 2 not connected
 * @param device_address - Stores equivalent hex device address.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
uint8_t ltc4306_identify_device_id(struct ltc4306_init_param init_param, 
	uint8_t adr0_connection, uint8_t adr1_connection, 
	uint8_t adr2_connection, uint8_t *device_address)
{
	//initiate variables
	uint8_t register_input = register_address_desc;
	uint8_t byte1 = 0;
	uint8_t byte2 = 0;

	if (adr2_connection == 0)
		byte2 = 0x80;
	else if (adr2_connection == 2)
		byte2 = 0x90;
	else if (adr2_connection == 1)
		byte2 = 0xA0;
	else
		return -EINVAL;
	
	if (adr1_connection == 2 && adr0_connection == 0)
		byte1 = 0x00;
	else if (adr1_connection == 1 && adr0_connection == 2)
		byte1 = 0x02;
	else if (adr1_connection == 2 && adr0_connection == 2)
		byte1 = 0x04;
	else if (adr1_connection == 2 && adr0_connection == 1)
		byte1 = 0x06;
	else if (adr1_connection == 0 && adr0_connection == 0)
		byte1 = 0x08;
	else if (adr1_connection == 1 && adr0_connection == 1)
		byte1 = 0x0A;
	else if (adr1_connection == 0 && adr0_connection == 2)
		byte1 = 0x0C;
	else if (adr1_connection == 0 && adr0_connection == 1)
		byte1 = 0x0E;
	else
	{
		if (adr2_connection == 1 && adr1_connection == 1 && adr0_connection == 0)
		{
			*device_address = 0xB0;
			return 0;
		}
		else if (adr2_connection == 0 && adr1_connection == 1 && adr0_connection == 0)
		{
			*device_address = 0xB2;
			return 0;
		}
		else if (adr2_connection == 2 && adr1_connection == 1 && adr0_connection == 0)
		{
			*device_address = 0xB4;
			return 0;
		}
		else
			return -EINVAL;
	}

	*device_address = byte2 | byte1;
	return 0;
}

/***************************************************************************//**
 * @brief Connects to a downstream bus. Bus logic state must be 1 for connection
 * 		to occur EXCEPT when conn_req is 1. Controller will connect to 
 * 		downstream bus regardless of its logic state if conn_req is 1.
 *
 * @param dev - The device structure.
 * @param connect_to_bus - Bus numner that the user wants to connect to.
 * 		(can be any value from 1 to 4) 
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_connect_to_downstream_channel(struct ltc4306_dev *dev, uint8_t connect_to_bus)
{
	uint8_t register_value;
	uint8_t bus_state_check;
	uint8_t conn_req;
	uint8_t temp_value;
	uint8_t ret;

	ltc4306_read_register_value(dev, LTC4306_CTRL_REG2, &conn_req);
	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG3, &register_value);
	if (ret)
		return ret;

	conn_req &= 0x20;
	temp_value = register_value;
	temp_value &= 0xF0;

	if (conn_req == 0)
	{
		switch (connect_to_bus)
		{
			case 1:
				ret = ltc4306_read_bus_logic_state(dev, 1, &bus_state_check);
				if (ret)
					return ret;
				if (bus_state_check == 0x08)
				{
					if (! (temp_value & 0x80))
						register_value |= 0x80;
					
					return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				}
				else					
					return -1;
				break;
			case 2:
				ret = ltc4306_read_bus_logic_state(dev, 2, &bus_state_check);
				if (ret)
					return ret;
				if (bus_state_check == 0x04)
				{
					if (! (temp_value & 0x40))
						register_value |= 0x40;
					
					return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				}
				else
					return -1;
				break;
			case 3:
				ret = ltc4306_read_bus_logic_state(dev, 3, &bus_state_check);
				if (ret)
					return ret;
				if (bus_state_check == 0x02)
				{
					if (! (temp_value & 0x20))
						register_value |= 0x20;
					
					return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				}
				else
					return -1;
				break;
			case 4:
				ret = ltc4306_read_bus_logic_state(dev, 4, &bus_state_check);
				if (ret)
					return ret;
				if (bus_state_check == 0x01)
				{
					if (! (temp_value & 0x10))
						register_value |= 0x10;
					
					return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				}
				else
					return -1;
				break;
			default:
				return -EINVAL;
		}
	}
	else
	{
		switch (connect_to_bus)
		{
			case 1:
				if (! (temp_value & 0x80))
					register_value |= 0x80;
				
				return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				break;
			case 2:
				if (! (temp_value & 0x40))
					register_value |= 0x40;
				
				return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				break;
			case 3:
				if (! (temp_value & 0x20))
					register_value |= 0x20;
				
				return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				break;
			case 4:
				if (! (temp_value & 0x10))
					register_value |= 0x10;
				
				return ltc4306_write_register_value(dev, LTC4306_CTRL_REG3, register_value);
				break;
			default:
				return -EINVAL;
		}
	}
}

/***************************************************************************//**
 * @brief Enables and disables upstream and downstream accelerator.
 *
 * @param dev - The device structure.
 * @param upstream_en - Set true to enable upstream rise time accelerator; false
 * 		to disable
 * @param downstream_en - Set true to enable downstream rise time accelerator;
 * 		false to disable.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_upstream_downstream_accelerator_en(struct ltc4306_dev *dev, 
	bool upstream_en, bool downstream_en)
{
	static uint8_t write_data;
	uint8_t temp = 0;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG1, &write_data);
	if (ret)
		return ret;

	temp = write_data;

	if (upstream_en)
	{
		if (! (temp & 0x80))
			write_data |= 0x80;
	}
	else
	{
		temp &= 0x80;
		if (temp == 0x80)
			write_data &= 0x7F;
	}

	if (downstream_en)
	{
		if (! (temp & 0x40))
			write_data |= 0x40;
	}
	else
	{
		temp &= 0x40;
		if (temp == 0x40)
			write_data &= 0xBF;
	}
	return ltc4306_write_register_value(dev, LTC4306_CTRL_REG1, write_data);
}

/***************************************************************************//**
 * @brief Enables Mass Write Adddress.
 *
 * @param dev - The device structure.
 * @param mass_write_en - Set true to enable mass write address; false to disable.
 * 		
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_mass_write_en(struct ltc4306_dev *dev, bool mass_write_en)
{
	uint8_t register_value;
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG2, &register_value);
	if (ret)
		return ret;		

	temp = register_value;
	temp &= 0x04;

	if (mass_write_en == true)
	{
		if (temp == 0x00)
			register_value |= 0x04;
	}
	else
	{
		if (temp == 0x04)
			register_value &= 0xFB;
	}
	return ltc4306_write_register_value(dev, LTC4306_MASS_WRITE_ADDR, register_value);
}

/***************************************************************************//**
 * @brief Configures GPIO1 and GPIO2 as input or output mode. If selected as
 * 		output mode, it can be configured as open-drain or push-pull.
 *
 * @param dev - The device structure.
 * @param gpio1_is_input - Set true to configure GPIO1 to input mode; 
 * 		false for output mode.
 * @param gpio2_is_input - Set true to configure GPIO2 to input mode; 
 * 		false for output mode.
 * @param gpio1_is_pushpull - Set true to configure GPIO1 output mode 
 * 		to push-pull; false for open-drain pull-down.
 * @param gpio2_is_pushpull - Set true to configure GPIO2 output mode 
 * 		to push-pull; false for open-drain pull-down.
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_gpio_mode_configure(struct ltc4306_dev *dev, bool gpio1_is_input, 
	bool gpio2_is_input, bool gpio1_is_pushpull, bool gpio2_is_pushpull)
{
	uint8_t register_value;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG2, &register_value);
	if (ret)
		return ret;
	
	if (gpio1_is_input)
		register_value &= 0x7F;
	else
	{
		register_value |= 0x80;
		if (gpio1_is_pushpull)
			register_value |= 0x10;
		else
			register_value &= 0xEF;
	}
	
	if (gpio2_is_input)
		register_value &= 0xBF;
	else
	{
		register_value |= 0x40;
		if (gpio2_is_pushpull)
			register_value |= 0x08;
		else
			register_value &= 0xF7;
	}
	
	return ltc4306_write_register_value(dev, LTC4306_CTRL_REG2, register_value);
}

/***************************************************************************//**
 * @brief Sets Connection Requirement bit.
 *
 * @param dev - The device structure.
 * @param connect_regardless - Set true to configure controller to connect to
 * 		downstream bus regardless of bus logic state; otherwise, set false.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_set_connection_requirement(struct ltc4306_dev *dev, bool connect_regardless)
{
	uint8_t register_value;
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG2, &register_value);
	if (ret)
		return ret;

	temp = register_value;
	
	if (connect_regardless)
	{
		if (! (temp & 0x20))
			register_value |= 0x20;
	}
	else
	{
		if (temp & 0x20)
			register_value &= 0xDF;
	}

	return ltc4306_write_register_value(dev, LTC4306_CTRL_REG2, register_value);
}

/***************************************************************************//**
 * @brief Reads bus logic state.
 *
 * @param dev - The device structure.
 * @param bus_number - Selects what bus to read (any number from 1 to 4)
 * @param is_high - Stores logic state of selected bus (true = high, false = low).
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_bus_logic_state(struct ltc4306_dev *dev, uint8_t bus_number, bool *is_high)
{
	uint8_t temp;
	uint8_t bus_logic_state;
	uint8_t ret;
	
	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG3, &temp);
	if (ret)
		return ret;

	bus_logic_state = temp;
	bus_logic_state &= 0x0F;

	switch (bus_number)
	{
		case 1:
			bus_logic_state &= 0x08;
			*is_high = bus_logic_state;
			return 0;
			break;
		case 2:
			bus_logic_state &= 0x04;
			*is_high = bus_logic_state;
			return 0;
			break;
		case 3:
			bus_logic_state &= 0x02;
			*is_high = bus_logic_state;
			return 0;
			break;
		case 4:
			bus_logic_state &= 0x01;
			*is_high = bus_logic_state;
			return 0;
			break;
		default:
			return -EINVAL;
	}
}

/***************************************************************************//**
 * @brief Reads Downstream Connected bit; indicates if upstream bus is
 * 		connected to any downstream buses.
 *
 * @param dev - The device structure.
 * @param is_high - stores Downstream Connected bit value (true = high, false = low).
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_downstream_connected_bit(struct ltc4306_dev *dev, bool *is_high)
{
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG0, &temp);
	if (ret)
		return ret;

	temp &= 0x80;
	*is_high = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Reads Alert Logic State of ALERT pins.
 *
 * @param dev - The device structure.
 * @param alert_pin_number - Sets ALERT pin to be read.
 * @param is_high - Stores value of selected ALERT pin (true = high, false = low).
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_alert_logic_state(struct ltc4306_dev *dev, uint8_t alert_pin_number, bool *is_high)
{
	uint8_t temp;
	uint8_t ret;

	if (alert_pin_number < 0 || alert_pin_number > 4)
		return -EINVAL;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG0, &temp);
	if (ret)
		return ret;
	
	switch (alert_pin_number)
	{
		case 1:
			temp &= 0x40;
			*is_high = temp;
			break;
		case 2:
			temp &= 0x20;
			*is_high = temp;
			break;
		case 3:
			temp &= 0x10;
			*is_high = temp;
			break;
		default:
			temp &= 0x08;
			*is_high = temp;
	}

	return 0;
}

/***************************************************************************//**
 * @brief Reads Failed Connection Attempt bit.
 *
 * @param dev - The device structure.
 * @param is_high - Stores value of Failed Connection Attempt bit (true = high,
 * 		false = low).
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_failed_connection_attempt_bit(struct ltc4306_dev *dev, bool *is_high)
{
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG0, &temp);
	if (ret)
		return ret;

	temp &= 0x04;
	*is_high = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Reads Latched Timeout bit.
 *
 * @param dev - The device structure.
 * @param is_high - Stores value of Latched Timeout bit (true = high, false = low).
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_latched_timeout_bit(struct ltc4306_dev *dev, bool *is_high)
{
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG0, &temp);
	if (ret)
		return ret;

	temp &= 0x02;
	*is_high = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Sets Timeout Mode bit 0 and bit 1.
 *
 * @param dev - The device structure.
 * @param timeout_mode_value - Input one of the following:
 * 		0b00	Timeout Disabled
 * 		0b01	Timeout after 30ns
 * 		0b10	Timeout after 15ms
 * 		0b11	Timeout after 7.5ms
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_set_timeout_mode(struct ltc4306_dev *dev, uint8_t timeout_mode_value)
{
	uint8_t register_value;
	uint8_t ret;

	if (timeout_mode_value != 0b00 || timeout_mode_value != 0b01 ||
		timeout_mode_value != 0b10 || timeout_mode_value != 0b11)
		return -EINVAL;
	
	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG2, &register_value);
	if (ret)
		return ret;

	register_value &= 0xFC;
	
	switch (timeout_mode_value)
	{
		case 0b11:
			register_value |= 0x11;
			break;
		case 0b10:
			register_value |= 0x10;
			break;
		case 0b01:
			register_value |= 0x01;
			break;
		default:
			return ltc4306_write_register_value(dev, LTC4306_CTRL_REG2, register_value);
	}
		
	return ltc4306_write_register_value(dev, LTC4306_CTRL_REG2, register_value);
}

/***************************************************************************//**
 * @brief Reads GPIO1 and GPIO 2 logic states.
 *
 * @param dev - The device structure.
 * @param register_value - Stores logic state value of GPIO1 and GPIO2.
 * 		(e.g. register_value = 0x01; 0 = GPIO1 state, 1 = GPIO2 state )
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_gpio_logic_state(struct ltc4306_dev *dev, uint8_t *register_value)
{
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG1, &temp);
	if (ret)
		return ret;

	temp &= 0x03;
	*register_value = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Sets GPIO Output Driver state.
 *
 * @param dev - The device structure.
 * @param gpio1_is_high - Sets GPIO1 driver state to either high or low
 * 		(true = high, false = low).
 * @param gpio2_is_high - Sets GPIO2 driver state to either high or low
 * 		(true = high, false = low).
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_write_gpio_output_state(struct ltc4306_dev *dev, bool gpio1_is_high, bool gpio2_is_high)
{
	uint8_t register_value;
	uint8_t temp;
	uint8_t ret;

	ret = ltc4306_read_register_value(dev, LTC4306_CTRL_REG1, &register_value);
	if (ret)
		return ret;

	register_value &= 0xCF;

	if (gpio1_is_high)
	{
		if (gpio2_is_high)
		{
			register_value |= 0x30;
			return ltc4306_write_register_value(dev, LTC4306_CTRL_REG1, register_value);
		}
		else
		{
			register_value |= 0x20;
			return ltc4306_write_register_value(dev, LTC4306_CTRL_REG1, register_value);
		}
	}
	else
	{
		if (gpio2_is_high)
		{
			register_value |= 0x10;
			return ltc4306_write_register_value(dev, LTC4306_CTRL_REG1, register_value);
		}
		else
			return ltc4306_write_register_value(dev, LTC4306_CTRL_REG1, register_value);
	}

}