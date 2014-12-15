#include <iostream>
// #include <unistd.h>
#include "multiwii.h"

#include <stdio.h>
#include <time.h>

#include <sys/select.h>

using namespace std;

int main()
{
    SERIAL s = serial_open("/dev/ttyUSB0", SERIAL_115200_BAUDRATE);

    if(s == -1)
        return perror("serial_open"), 1;

    {
        timespec t; t.tv_sec = 2; t.tv_nsec=0;
        (void) nanosleep(&t, NULL);
    }

    if(!multiwii_init(s))
        return perror("multiwii_init"), 1;

    if(!multiwii_exec(NULL))
        printf("error\n");

    serial_close(s);

    return 0;
}

