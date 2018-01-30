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


#include <stdlib.h>   
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>

#define SERIAL_SIZE_OF_DATA_BUFFER 5
#define MAX_UART_DEVICE_NUM      6

const char *serial_device_path[MAX_UART_DEVICE_NUM] = {
	"/dev/tty-1", "/dev/tty-2", "/dev/tty-3",
	"/dev/tty-4", "/dev/tty-5", "/dev/tty-6"
};

int dspal_tester_serial_read_with_small_buffer(void)
{
	int result = 0;
	int num_bytes_read = 0;
	char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	int fd;
	int max_read_bytes = 4;
	int devid = 3;
	int ftemp = 0;

	printf("beginning serial read with small buffer test");

	fd = open(serial_device_path[devid], O_RDWR);
	printf("open %s O_RDWR mode %s", serial_device_path[devid],
		 (fd < 0) ? "fail" : "succeed");

	if (fd < 0) {
		result = -1;
		goto exit;
	}

	// wait 250ms to ensure the data is received in the loopback
	usleep(250000);
	memset(rx_buffer, 0, SERIAL_SIZE_OF_DATA_BUFFER);
	num_bytes_read = read(fd, rx_buffer, max_read_bytes);
	char* testig = rx_buffer;

	ftemp = atoi(rx_buffer);
	printf("ShowMeWhatYouGot serial_test_imp.c dspal_tester_serial_read_with_small_buffer: %i",ftemp);
	return ftemp;

exit:

	if (fd >= 0) {
		close(fd);
	}

	return ftemp;
}


int  dspal_tester_serial_test(void)
{
	int result = 0;

	result = dspal_tester_serial_read_with_small_buffer();
	printf("ShowMeWhatYouGot serial_test_imp.c dspal_tester_serial_test: %i",result);

	return result;
}
