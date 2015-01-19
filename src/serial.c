#include "serial.h"

#include <unistd.h>     /* io */
#include <fcntl.h>      /* fd_set */
#include <termios.h>    /* serial_open */
#include <stdio.h>      /* perror */
#include <sys/ioctl.h>  /* ioctl */
#include <sys/select.h>     /* select */

STATIC_ASSERT(sizeof(duint8_t) * 8 == 8, UINT);
STATIC_ASSERT(sizeof(duint16_t) * 8 == 16, UINT);
STATIC_ASSERT(sizeof(duint32_t) * 8 == 32, UINT);
STATIC_ASSERT(sizeof(duint64_t) * 8 == 64, UINT);

#ifdef __cplusplus
extern "C" {
#endif /*cpp*/

SERIAL serial_open(const char* serialport, int baudrate)
{
    SERIAL fd;
    struct termios toptions;

    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
        goto cleanup;

    if (CHK_IO_FAILED(tcgetattr(fd, &toptions)))
        goto cleanup;

    switch (baudrate) {
    case SERIAL_9600_BAUDRATE:
        baudrate = B9600;
        break;
    case SERIAL_19200_BAUDRATE:
        baudrate = B19200;
        break;
    case SERIAL_38400_BAUDRATE:
        baudrate = B38400;
        break;
    case SERIAL_57600_BAUDRATE:
        baudrate = B57600;
        break;
    case SERIAL_115200_BAUDRATE:
    default:
        baudrate = B115200;
        break;
    }

    if(CHK_IO_FAILED(cfsetispeed(&toptions, baudrate)))
        goto cleanup;
    if(CHK_IO_FAILED(cfsetospeed(&toptions, baudrate)))
        goto cleanup;

    toptions.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    /*toptions.c_oflag = 0;*/

    toptions.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    toptions.c_cflag &= ~(CSIZE | PARENB);
    toptions.c_cflag |= CS8;
    toptions.c_cflag |= CLOCAL;

    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 10;


    /*apply options*/
    if (CHK_IO_FAILED(tcsetattr(fd, TCSAFLUSH, &toptions)))
        goto cleanup;

    return fd;

cleanup:
    perror("serial_open");
    (void) close(fd);
    return -1;
}

void serial_close(SERIAL fd)
{
    return (void) close(fd);
}

diostatus_t serial_write(SERIAL fd, const char* data, unsigned long len)
{
    diostatus_t n = write(fd, data, len);
    if(CHK_IO_FAILED(n))
        return -1;

    if(CHK_IO_FAILED(tcdrain(fd)))
        return -1;

    if (n != (diostatus_t)len)
        return -1;

    return n;
}

diostatus_t serial_write_byte(SERIAL fd, const char data)
{
    diostatus_t n = write(fd, &data, 1);
    if(CHK_IO_FAILED(n))
        return -1;

    if(CHK_IO_FAILED(tcdrain(fd)))
        return -1;

    return (diostatus_t)(n == 1);
}

diostatus_t serial_read(SERIAL fd, char* buf, unsigned long len)
{
    return (diostatus_t)read(fd, buf, len);
}

diostatus_t serial_available(SERIAL fd)
{
    unsigned long int nbytes = 0;
    if(-1 == ioctl(fd, FIONREAD, &nbytes))
        return -1;

    return nbytes;
}

diostatus_t serial_available_for(SERIAL port, int microseconds)
{
    fd_set fdset;
    fd_set* input = &fdset;
    struct timeval timeout;

    FD_ZERO(input);
    FD_SET(port, input);

    timeout.tv_sec  = (microseconds - (microseconds % 1000)) / 1000;
    timeout.tv_usec = microseconds % 1000;

    diostatus_t n = select(port + 1, input, NULL, NULL, &timeout);

    if (CHK_IO_FAILED(n)) {
        return perror("select failed"), n;
    } else if (CHK_IO_TIMEOUT(n)) {
        /* timeout */
        return n;
    } else {
        if (!FD_ISSET(port, input))
            return 0;

        return n;
    }
}

#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/
