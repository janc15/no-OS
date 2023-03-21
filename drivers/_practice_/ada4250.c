/***************************************************************************//**
 *   @file   ada4250.c
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

#include "ada4250.h"
#include "no_os_error.h"
#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/**************************** Types Definitions *******************************/
/******************************************************************************/

/**
 * @brief Demux specific SPI platform ops structure
 */
const struct no_os_spi_platform_ops ada4250_platform_ops = {
	.init = ada4250_init,
	.remove = ada4250_remove,
	.write_and_read = ada4250_write_and_read
};

/**
 * @brief Initialize the SPI demux layer.
 * @param desc - The SPI descriptor.
 * @param param - The structure that contains the SPI parameters.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ada4250_init(struct no_os_spi_desc **desc,
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
 * @brief Free the resources allocated by ada4250_init().
 * @param desc - The SPI descriptor.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t ada4250_remove(struct no_os_spi_desc *desc)
{
	if (!desc)
		return -1;

	if (no_os_spi_remove(desc->extra))
		return -1;

	free(desc);

	return 0;
}

/***************************************************************************//**
 * @brief Sends data to ADA4250.
 *
 * @param dev         - The device structure.
 * @param control_reg - Value of control register.
 *                     Example:
 *                     ADA4250_INT | ADA4250_LDAC | ADA4250_A  - enables internal
 *                     reference and loads DAC A input register from shift
 *                     register and updates both DAC A and DAC B DAC registers.
 * @param data_val    - Value of data register.
 *
 * @return None.
*******************************************************************************/
int ada4250_write(struct ada4250_dev *dev,
		  uint16_t control_reg,
		  uint8_t data_val)
{

    uint8_t write_data[2];

    if (control_reg < 0x00 || (control_reg > 0x05 && control_reg < 0x18) || 
        control_reg == 0x03 || control_reg > 0x1A)
        return -1;

    write_data[0] = data_val;
    write_data[1] = control_reg;

	return no_os_spi_write_and_read(dev->spi_desc, write_data, 2);
}

/***************************************************************************//**
 * @brief Sends data to ADA4250.
 *
 * @param dev         - The device structure.
 * @param control_reg - Value of control register.
 *                     Example:
 *                     ADA4250_INT | ADA4250_LDAC | ADA4250_A  - enables internal
 *                     reference and loads DAC A input register from shift
 *                     register and updates both DAC A and DAC B DAC registers.
 * @param data_reg    - Value of data register.
 *
 * @return None.
*******************************************************************************/
int ada4250_read(struct ada4250_dev *dev,
		  uint16_t control_reg,
		  uint8_t *data_val)
{
	uint8_t read_data[2];

    if (control_reg < 0x00 || (control_reg > 0x05 && control_reg < 0x18) || 
        control_reg == 0x03 || control_reg > 0x1A)
        return -1;

    control_reg |= 1 >> 1;
    
	read_data[0] = control_reg;

	ret = no_os_spi_write_and_read(dev->spi_desc, read_data, 1);

    if (ret)
        return ret;

    *data_val = read_data[0];

    return 0;
}
