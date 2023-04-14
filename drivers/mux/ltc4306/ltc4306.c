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
 * @param dev - The device structure.
 * @param register_address - Address of the register.
 * @param register_value - Data value to write.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_write_register_value(struct ltc4306_dev *dev,
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
 * @param dev - The device structure.
 * @param register_address - Address of the register.
 * @param register_value - Stores data read from the register.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/

int ltc4306_read_register_value(struct ltc4306_dev *dev,
				   uint8_t register_address, uint8_t *register_value)
{
	static uint8_t read_data[2]   = {0, 0};
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

/***************************************************************************//**
 * @brief Gives the equivalent hex device address based on the input combination
 * 		of ADR0, ADR1, and ADR2. (27 possible device address)
 *
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 * @param adr0_connection - Used to configure slave address. Input based on 
 * 		ADR0 connection: "l" low logic, "h" high logic, "nc" not connected
 * @param adr1_connection - Used to configure slave address. Input based on 
 * 		ADR1 connection: "l" low logic, "h" high logic, "nc" not connected
 * @param adr2_connection - Used to configure slave address. Input based on 
 * 		ADR2 connection: "l" low logic, "h" high logic, "nc" not connected
 * @param device_address - Stores equivalent hex device address.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
uint8_t ltc4306_identify_device_id(struct ltc4306_init_param init_param, 
	char adr0_connection, char adr1_connection, char adr2_connection, *device_address)
{
	//initiate variables
	uint8_t register_input = register_address_desc;
	uint8_t byte1 = 0;
	uint8_t byte2 = 0;

	if (adr2_connection == "l")
		byte2 = 0x80;
	else if (adr2_connection == "nc")
		byte2 = 0x90;
	else if (adr2_connection == "h")
		byte2 = 0xA0;
	else
		return -EINVAL;
	
	if (adr1_connection == "nc" && adr0_connection == "l")
		byte1 = 0x00;
	else if (adr1_connection == "h" && adr0_connection == "nc")
		byte1 = 0x02;
	else if (adr1_connection == "nc" && adr0_connection == "nc")
		byte1 = 0x04;
	else if (adr1_connection == "nc" && adr0_connection == "h")
		byte1 = 0x06;
	else if (adr1_connection == "l" && adr0_connection == "l")
		byte1 = 0x08;
	else if (adr1_connection == "h" && adr0_connection == "h")
		byte1 = 0x0A;
	else if (adr1_connection == "l" && adr0_connection == "nc")
		byte1 = 0x0C;
	else if (adr1_connection == "l" && adr0_connection == "h")
		byte1 = 0x0E;
	else
	{
		if (adr2_connection == "h" && adr1_connection == "h" && adr0_connection == "l")
		{
			*device_address = 0xB0;
			return 0;
		}
		else if (adr2_connection == "l" && adr1_connection == "h" && adr0_connection == "l")
		{
			*device_address = 0xB2;
			return 0;
		}
		else if (adr2_connection == "nc" && adr1_connection == "h" && adr0_connection == "l")
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

	ltc4306_read_register_value(dev, 0x02, &conn_req);
	ltc4306_read_register_value(dev, 0x03, &register_value);

	conn_req &= 0x20;
	temp_value = register_value;
	temp_value &= 0xF0;

	if (conn_req == 0)
	{
		// first if statement masks bit depending on what bus you want to connect
		if (connect_to_bus == 1)
		{
			ltc4306_read_bus_logic_state(dev, 1, &bus_state_check);
			//checks bus logic state, if high, will set bus state fet high to connect to bus
				if (bus_state_check == 0x08)
				{
					temp_value &= 0x80;
					if (temp_value == 0x00)
						register_value |= 0x80;
					
					return ltc4306_write_register_value(dev, 0x03, register_value);
				}
				else
				{
					printf("Bus 1 logic state is 0. Must be logic 1 to connect.");
					return 0;
				}
		}
		else if (connect_to_bus == 2)
		{
			ltc4306_read_bus_logic_state(dev, 2, &bus_state_check);
				if (bus_state_check == 0x04)
				{
					temp_value &= 0x40;
					if (temp_value == 0x00)
						register_value |= 0x40;
					
					return ltc4306_write_register_value(dev, 0x03, register_value);
				}
				else
				{
					printf("Bus 2 logic state is 0. Must be logic 1 to connect.");
					return 0;
				}
		}
		else if (connect_to_bus == 3)
		{
			ltc4306_read_bus_logic_state(dev, 3, &bus_state_check);
				if (bus_state_check == 0x02)
				{
					temp_value &= 0x20;
					if (temp_value == 0x00)
						register_value |= 0x20;
					
					return ltc4306_write_register_value(dev, 0x03, register_value);
				}
				else
				{
					printf("Bus 3 logic state is 0. Must be logic 1 to connect.");
					return 0;
				}
		}
		else if (connect_to_bus == 4)
		{
			ltc4306_read_bus_logic_state(dev, 4, &bus_state_check);
				if (bus_state_check == 0x01)
				{
					temp_value &= 0x10;
					if (temp_value == 0x00)
						register_value |= 0x10;
					
					return ltc4306_write_register_value(dev, 0x03, register_value);
				}
				else
				{
					printf("Bus 4 logic state is 0. Must be logic 1 to connect.");
					return 0;
				}
		}
		else
			return -EINVAL;
	}
	else
	{
		if (connect_to_bus == 1)
		{
			temp_value &= 0x80;
			if (temp_value == 0x00)
				register_value |= 0x80;
			
			return ltc4306_write_register_value(dev, 0x03, register_value);
		}
		else if (connect_to_bus == 2)
		{
			temp_value &= 0x40;
			if (temp_value == 0x00)
				register_value |= 0x40;
			
			return ltc4306_write_register_value(dev, 0x03, register_value);
		}
		else if (connect_to_bus == 3)
		{
			temp_value &= 0x20;
			if (temp_value == 0x00)
				register_value |= 0x20;
			
			return ltc4306_write_register_value(dev, 0x03, register_value);
		}
		else if (connect_to_bus == 4)
		{
			temp_value &= 0x80;
			if (temp_value == 0x00)
				register_value |= 0x80;
			
			return ltc4306_write_register_value(dev, 0x03, register_value);
		}
		else
			return -EINVAL;
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
int ltc4306_upstream_downstream_accelerator_en(struct ltc4306_dev *dev, bool upstream_en, bool downstream_en)
{
	static uint8_t write_data;
	uint8_t temp = 0;

	ltc4306_read_register_value(dev, 0x01, &write_data);
	temp = write_data;

	if (upstream_en == true)
	{
		temp &= 0x80;
		if (temp == 0x00)
			write_data |= 0x80;
	}
	else
	{
		temp &= 0x80;
		if (temp == 0x80)
			write_data &= 0x7F;
	}

	if (downstream_en == true)
	{
		temp &= 0x40;
		if (temp == 0x00)
			write_data |= 0x40;
	}
	else
	{
		temp &= 0x40;
		if (temp == 0x40)
			write_data &= 0xBF;
	}
	return ltc4306_write_register_value(dev, 0x01, write_data);
}

/***************************************************************************//**
 * @brief Performs Alert Response Address Protocol.
 *
 * @param dev - The device structure.
 * @param device_address - Address of slave device that sent interrupt signal.
 * @param register_value - Stores data read from device address.
 *
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_alert_response(struct ltc4306_dev *dev, uint8_t device_address, uint8_t *register_value)
{
	uint8_t read_data[2] = {0, 0};
	uint8_t ret;
	read_data[0] = 0x19;
	read_data[1] = device_address;

	ret = no_os_i2c_write(dev->i2c_desc, read_data, 1, 0);
	if (ret)
		return ret;

	*register_value = no_os_i2c_read(dev->i2c_desc, read_data, 1, 1);

	return register_value;
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

	ltc4306_read_register_value(dev, 0x02, &register_value);

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
	return ltc4306_write_register_value(dev, 0xBA, register_value);
}

/***************************************************************************//**
 * @brief Configures GPIO1 and GPIO2 as input or output mode. If selected as
 * 		output mode, it can be configured as open-drain or push-pull.
 *
 * @param dev - The device structure.
 * @param gpio1_mode_config - Set true to configure GPIO1 to input mode; 
 * 		false for output mode.
 * @param gpio2_mode_config - Set true to configure GPIO2 to input mode; 
 * 		false for output mode.
 * @param gpio1_output_mode_config - Set true to configure GPIO1 output mode 
 * 		to push-pull; false for open-drain pull-down.
 * @param gpio2_output_mode_config - Set true to configure GPIO2 output mode 
 * 		to push-pull; false for open-drain pull-down.
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_gpio_mode_configure(struct ltc4306_dev *dev, bool gpio1_mode_config, bool gpio2_mode_config, 
								bool gpio1_output_mode_config, bool gpio2_output_mode_config)
{
	uint8_t register_address = 0x02;
	uint8_t register_value = 0;

	register_value = ltc4306_read_register_value(dev, register_address);
	
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
	
	return ltc4306_write_register_value(dev, register_address, register_value);
}

/***************************************************************************//**
 * @brief Sets Connection Requirement bit.
 *
 * @param dev - The device structure.
 * @param connection_requirement_bit - Set true to configure controller to
 * 		connect to downstream bus regardless of bus logic state; otherwise,
 * 		set false.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_set_connection_requirement(struct ltc4306_dev *dev, bool connection_requirement_bit)
{
	uint8_t register_value;
	uint8_t temp;

	ltc4306_read_register_value(dev, 0x02, &register_value);
	temp = register_value;
	temp &= 0x20;
	
	if (connection_requirement_bit)
	{
		if (temp == 0x00)
			register_value |= 0x20;
	}
	else
	{
		if (temp == 0x20)
			register_value &= 0xDF;
	}

	return ltc4306_write_register_value(dev, 0x02, register_value);
}

/***************************************************************************//**
 * @brief Reads bus logic state.
 *
 * @param dev - The device structure.
 * @param bus_number - Selects what bus to read (any number from 1 to 4)
 * @param logic_state - Stores logic state of selected bus.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_bus_logic_state(struct ltc4306_dev *dev, uint8_t bus_number, uint8_t *logic_state)
{
	uint8_t temp;
	uint8_t bus_logic_state;

	ltc4306_read_register_value(dev, 0x03, &temp);
	bus_logic_state = temp;
	bus_logic_state &= 0x0F;

	if (bus_number == 1)
	{
		bus_logic_state &= 0x08;
		*logic_state = bus_logic_state;
		return 0;
	}
	else if (bus_number == 2)
	{
		bus_logic_state &= 0x04;
		*logic_state = bus_logic_state;
		return 0;
	}
	else if (bus_number == 3)
	{
		bus_logic_state &= 0x02;
		*logic_state = bus_logic_state;
		return 0;
	}
	elseif (bus_number == 4)
	{
		bus_logic_state &= 0x01;
		*logic_state = bus_logic_state;
		return 0;
	}
	else
		return -EINVAL;
}

/***************************************************************************//**
 * @brief Reads Downstream Connected bit; indicates if upstream bus is
 * 		connected to any downstream buses.
 *
 * @param dev - The device structure.
 * @param register_value - stores Downstream Connected bit value.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_downstream_connected_bit(struct ltc4306_dev *dev, uint8_t *register_value)
{
	uint8_t temp;

	ret = ltc4306_read_register_value(dev, 0x00, &temp);
	temp &= 0x80;
	*register_value = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Reads Alert Logic State of ALERT pins.
 *
 * @param dev - The device structure.
 * @param alert_pin_number - Sets ALERT pin to be read.
 * @param register_value - Stores value of selected ALERT pin.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_alert_logic_state(struct ltc4306_dev *dev, uint8_t alert_pin_number, uint8_t *register_value)
{
	uint8_t temp;

	if (alert_pin_number < 0 || alert_pin_number > 4)
		return -EINVAL;

	ltc4306_read_register_value(dev, 0x00, &temp);
	
	if (alert_pin_number == 1)
	{
		temp &= 0x40;
		*register_value = temp;
	}
	else if (alert_pin_number == 2)
	{
		temp &= 0x20;
		*register_value = temp;
	}
	else if (alert_pin_number == 3)
	{
		temp &= 0x10;
		*register_value = temp;
	}
	else
	{
		temp &= 0x08;
		*register_value = temp;
	}

	return 0;
}

/***************************************************************************//**
 * @brief Reads Failed Connection Attempt bit.
 *
 * @param dev - The device structure.
 * @param register_value - Stores value of Failed Connection Attempt bit.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_failed_connection_attempt_bit(struct ltc4306_dev *dev, uint8_t *register_value)
{
	uint8_t temp;

	ltc4306_read_register_value(dev, 0x00, &temp);
	temp &= 0x04;
	*register_value = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Reads Latched Timeout bit.
 *
 * @param dev - The device structure.
 * @param register_value - Stores value of Latched Timeout bit.
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_read_latched_timeout_bit(struct ltc4306_dev *dev, uint8_t *register_value)
{
	uint8_t temp;

	ltc4306_read_register_value(dev, 0x00, &temp);
	temp &= 0x02;
	*register_value = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Sets Timeout Mode bit 0 and bit 1.
 *
 * @param dev - The device structure.
 * @param timeout_mode_value - Input one of the following:
 * 		0x00	Timeout Disabled
 * 		0x01	Timeout after 30ns
 * 		0x10	Timeout after 15ms
 * 		0x11	Timeout after 7.5ms
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_set_timeout_mode(struct ltc4306_dev *dev, uint8_t timeout_mode_value)
{
	uint8_t register_value;

	if (timeout_mode_value != 0x00 || timeout_mode_value != 0x01 ||
		timeout_mode_value != 0x10 || timeout_mode_value != 0x11)
		return -EINVAL;
	
	ltc4306_read_register_value(dev, 0x02, &register_value);
	register_value &= 0xFC;
	
	if (timeout_mode_value == 0x11)
		register_value |= 0x11;
	else if (timeout_mode_value == 0x10)
		register_value |= 0x10;
	else if (timeout_mode_value == 0x01)
		register_value |= 0x01;
	else 
		return ltc4306_write_register_value(dev, 0x02, register_value);
		
	return ltc4306_write_register_value(dev, 0x02, register_value);
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

	ltc4306_read_register_value(dev, 0x01, &temp);
	temp &= 0x03;
	*register_value = temp;

	return 0;
}

/***************************************************************************//**
 * @brief Sets GPIO Output Driver state.
 *
 * @param dev - The device structure.
 * @param gpio1_gpio2_value - Input sets GPIO driver state.
 * 		(e.g. gpio1_gpio2_value = 0x01; 0 = GPIO1 state, 1 = GPIO2 state)
 * 
 * @return 0 in case of success, negative error code otherwise.
*******************************************************************************/
int ltc4306_write_gpio_output_state(struct ltc4306_dev *dev, uint8_t gpio1_gpio2_value)
{
	uint8_t register_value;
	uint8_t temp;

	ltc4306_read_register_value(dev, 0x01, &register_value);
	register_value &= 0xCF;

	if (gpio1_gpio2_value == 0x00)
		return ltc4306_write_register_value(dev, 0x01, register_value);
	else if (gpio1_gpio2_value == 0x01)
	{
		register_value |= 0x10;
		return ltc4306_write_register_value(dev, 0x01, register_value);
	}
	else if (gpio1_gpio2_value == 0x10)
	{
		register_value |= 0x20;
		return ltc4306_write_register_value(dev, 0x01, register_value);
	}
	else if (gpio1_gpio2_value == 0x11)
	{
		register_value |= 0x30;
		return ltc4306_write_register_value(dev, 0x01, register_value);
	}
	else
		return -EINVAL;
}