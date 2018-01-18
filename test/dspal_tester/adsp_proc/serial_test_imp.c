/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#include <platform.h>

#include "test_utils.h"
#include "test_status.h"

#define SERIAL_SIZE_OF_DATA_BUFFER 128

/**
 * Snapdragon Flight DSP supports up to 6 UART devices. However, the actual
 * number of available UART devices may vary depending on the board schematic
 * design. On Snapdragon Flight reference platform, 4 UART ports are available
 * for communication to external serial devices.
 * To make the serial test work for your board, please make sure the following
 * are compliant to your set up:
 * - set NUM_UART_DEVICE_ENABLED with the number of UART ports available
 * - make sure to define UART-BAM mappings in /usr/share/data/adsp/blsp.config
 *   See snapdragon flight user guide for details.
 */
#define MAX_UART_DEVICE_NUM      1

const char *serial_device_path[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-4"
};

int serial_fildes[MAX_UART_DEVICE_NUM] = { -1};
char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];

/**
* @brief Test serial read with small buffer
*
* @par Detailed Description:
* This test case is testing the scenario in which the serial bus receives X
* bytes. User calls read() to read the data and passes in a small buffer.
* read() should return -EINVAL error code in this caes
*
* Test:
* 1) Open the serial device /dev/tty-1
* 2) Write very long bytes to the serial device
* 3) wait for 100ms to make sure the loopback data is received
* 3) read() with 10 byte buffer and check the return value
* 5) Close serial device
*
* @return
* - SUCCESS
*/
char* dspal_tester_serial_read_with_small_buffer(void)
{
	int result = SUCCESS;
	int num_bytes_read = 0;
	int fd;
	int max_read_bytes = 20;
	int devid = 0;

	LOG_INFO("beginning serial read with small buffer test");

	fd = open(serial_device_path[devid], O_RDWR);
	LOG_INFO("open %s O_RDWR mode %s", serial_device_path[devid],
		 (fd < SUCCESS) ? "fail" : "succeed");

	if (fd < SUCCESS) {
		result = ERROR;
		goto exit;
	}

	// wait 100ms to ensure the data is received in the loopback
	usleep(500000);
	memset(rx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
	num_bytes_read = read(fd, rx_buffer, max_read_bytes);

	if (num_bytes_read == -1) {
		LOG_DEBUG("%s read() with small buffer return expected error code -1",
			  serial_device_path[devid]);
	} else {
		LOG_ERR("%s read() return: %d, expected -1",
			serial_device_path[devid], num_bytes_read);
	}
	LOG_ERR("ShowMeWhatYouGot: %s", rx_buffer)

exit:

	if (fd >= SUCCESS) {
		close(fd);
	}

	LOG_INFO("serial read with small buffer test %s",
		 result == SUCCESS ? "PASSED" : "FAILED");

	return rx_buffer;

}

/**
* @brief Runs all the serial tests and returns 1 aggregated result.
*
* @return
* SUCCESS ------ All tests pass
* ERROR -------- One or more tests failed
*/
char* dspal_tester_serial_test(void)
{
	char* result;
	result = dspal_tester_serial_read_with_small_buffer();
	LOG_ERR("ShowMeWhatYouGot %s",result);
	return 0;
}
