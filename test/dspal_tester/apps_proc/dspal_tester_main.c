
#include <stdlib.h>   
#include <stdint.h>
#include <string.h>
#include <stdio.h>

int main()
{

	printf("Starting DSPAL tests \n"); 
    int status ;
    status = dspal_tester_serial_test();
    printf("ShowMeWhatYouGot dspal_tester_main.c main: %i\n", status);
	return 0;
}

