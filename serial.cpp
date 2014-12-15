#include "serial.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h> // ioctl

STATIC_ASSERT(sizeof(duint8_t) * 8 == 8, UINT);
STATIC_ASSERT(sizeof(duint16_t) * 8 == 16, UINT);
STATIC_ASSERT(sizeof(duint32_t) * 8 == 32, UINT);
STATIC_ASSERT(sizeof(duint64_t) * 8 == 64, UINT);

SERIAL serial_open(const char* serialport, int baudrate)
/*
{
    struct termios toptions;
    int fd;
    fd = open(serialport, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("init_serialport: Unable to open port ");
        return -1;
    }

    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }

    speed_t brate = baudrate; // let you override switch below if needed
    switch (baudrate) {
    case 4800:
        brate = B4800;
        break;
    case 9600:
        brate = B9600;
        break;
    case 19200:
        brate = B19200;
        break;
    case 38400:
        brate = B38400;
        break;
    case 57600:
        brate = B57600;
        break;
    case 115200:
        brate = B115200;
        break;
    case 460800:
        brate = B460800;
        break;
    }

    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN] = 1;
    toptions.c_cc[VTIME] = 10;
    if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}
*/
{
    SERIAL fd;
    struct termios toptions;

    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        perror("init_serialport: Unable to open port ");
        return -1;
    }

    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        (void) close(fd);
        return -1;
    }

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
    cfsetispeed(&toptions, baudrate);
    cfsetospeed(&toptions, baudrate);

    toptions.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //toptions.c_oflag = 0;

    toptions.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    toptions.c_cflag &= ~(CSIZE | PARENB);
    toptions.c_cflag |= CS8;
    toptions.c_cflag |= CLOCAL;

    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 10;


    // apply options
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        (void) close(fd);
        return -1;
    }

    return fd;
}
// */

void serial_close(SERIAL fd)
{
    return (void) close(fd);
}

int serial_write(SERIAL fd, const char* data, unsigned long len)
{
    int n = write(fd, data, len);
    (void) tcdrain(fd);

    if (n != (int)len)
        return -1;

    return n;
}


int serial_write(SERIAL fd, const char data)
{
    int n = write(fd, &data, 1);
    (void) tcdrain(fd);

    return n;
}

int serial_read(SERIAL fd, char* buf, unsigned long len)
{
    //fcntl(fd, F_SETFL, 0);
    int ret = read(fd, buf, len);
    //fcntl(fd, F_SETFL, FNDELAY);

    return ret;
}

int serial_available(SERIAL fd)
{
    int nbytes = 0;
    (void) ioctl(fd, FIONREAD, &nbytes);

    return nbytes;
}

bool serial_available(SERIAL port, int microseconds)
{
    fd_set fdset;
    fd_set* input = &fdset;
    struct timeval timeout;

    FD_ZERO(input);
    FD_SET(port, input);

    timeout.tv_sec  = (microseconds - (microseconds % 1000)) / 1000;
    timeout.tv_usec = microseconds % 1000;

    int n = select(port + 1, input, NULL, NULL, &timeout);

    if (n < 0) {
        perror("select failed");
        return false;
    } else if (n == 0) {
        // timeout
        return false;
    } else {
        if (!FD_ISSET(port, input))
            return false;

        return true;
    }
}
