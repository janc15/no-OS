/***************************************************************************//**
 *   @file   adf4371.c
 *   @brief  Implementation of the SPI Demux Interface
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#include "adf4371.h"
#include "no_os_error.h"
#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/**************************** Types Definitions *******************************/
/******************************************************************************/

/**
 * @brief Demux specific SPI platform ops structure
 */
const struct no_os_spi_platform_ops adf4371_platform_ops = {
	.init = adf4371_init,
	.remove = adf4371_remove,
	.write_and_read = adf4371_write_and_read
};

/**
 * @brief Initialize the SPI demux layer.
 * @param desc - The SPI descriptor.
 * @param param - The structure that contains the SPI parameters.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adf4371_init(struct no_os_spi_desc **desc,
		       const struct no_os_spi_init_param *param)
{
	int32_t ret;

	struct no_os_spi_desc *descriptor;
	struct no_os_spi_desc *spi_dev_desc;
	struct no_os_spi_init_param *spi_dev_param;

	if (!param)
		return -1;

	descriptor = (struct no_os_spi_desc *)calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -1;

	descriptor->chip_select = param->chip_select;
	descriptor->max_speed_hz = param->max_speed_hz;
	descriptor->mode = param->mode;

	spi_dev_param = param->extra;

	ret = no_os_spi_init(&spi_dev_desc, spi_dev_param);
	if (ret != 0) {
		free(descriptor);
		return -1;
	}

	(descriptor->extra) = spi_dev_desc;

	*desc = descriptor;

	return ret;
}

/**
 * @brief Free the resources allocated by adf4371_init().
 * @param desc - The SPI descriptor.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adf4371_remove(struct no_os_spi_desc *desc)
{
	if (!desc)
		return -1;

	if (no_os_spi_remove(desc->extra))
		return -1;

	free(desc);

	return 0;
}

/***************************************************************************//**
 * @brief Sends data to AD3271.
 *
 * @param dev         - The device structure.
 * @param control_reg - Value of control register.
 *                     Example:
 *                     AD3271_INT | AD3271_LDAC | AD3271_A  - enables internal
 *                     reference and loads DAC A input register from shift
 *                     register and updates both DAC A and DAC B DAC registers.
 * @param data_val    - Value of data register.
 *
 * @return None.
*******************************************************************************/
int adf4371_write(struct adf4371_dev *dev,
		  uint16_t control_reg,
		  uint8_t data_val)
{

    uint8_t write_data[3];

	if (control_reg < 0x00 || control_reg == 0x02 || (control_reg > 0x06 && control_reg < 0x10) || 
        control_reg == 0x13 || control_reg == 0x21 || control_reg == 0x29 || (control_reg > 0x3A && control_reg < 0x3D) ||
		(control_reg > 0x42 && control_reg < 0x47) || (control_reg > 0x47 && control_reg < 0x52) || 
		(control_reg > 0x52 && control_reg < 0x6E) || (control_reg > 0x73 && control_reg < 0x7C) || control_reg > 0x7C)
        return -1;

    write_data[0] = control_reg & 0xFF;
    write_data[1] = control_reg >> 8;
    write_data[2] = data_val;

	return no_os_spi_write_and_read(dev->spi_desc, write_data, 3);
}

/***************************************************************************//**
 * @brief Sends data to AD3271.
 *
 * @param dev         - The device structure.
 * @param control_reg - Value of control register.
 *                     Example:
 *                     AD3271_INT | AD3271_LDAC | AD3271_A  - enables internal
 *                     reference and loads DAC A input register from shift
 *                     register and updates both DAC A and DAC B DAC registers.
 * @param data_reg    - Value of data register.
 *
 * @return None.
*******************************************************************************/
int adf4371_read(struct adf4371_dev *dev,
		  uint16_t control_reg,
		  uint8_t *data_val)
{
	uint8_t read_data[2];

	if (control_reg < 0x00 || control_reg == 0x02 || (control_reg > 0x06 && control_reg < 0x10) || 
        control_reg == 0x13 || control_reg == 0x21 || control_reg == 0x29 || (control_reg > 0x3A && control_reg < 0x3D) ||
		(control_reg > 0x42 && control_reg < 0x47) || (control_reg > 0x47 && control_reg < 0x52) || 
		(control_reg > 0x52 && control_reg < 0x6E) || (control_reg > 0x73 && control_reg < 0x7C) || control_reg > 0x7C)
        return -1;

	read_data[0] = control_reg;
    read_data[1] = (control_reg >> 8) | 0x80;

	ret = no_os_spi_write_and_read(dev->spi_desc, read_data, 2);

    if (ret)
        return ret;

    *data_val = read_data[0];

    return 0;
}