#ifndef SERIAL_H
#define SERIAL_H

#include "dlib.hxx"

#ifdef __cplusplus
extern "C" {
#endif /*cpp*/

typedef int SERIAL;
typedef int diostatus_t;

#define D_IO_EMPTY \
    ((diostatus_t)0)

#define CHK_IO_FAILED(IO) \
    ((diostatus_t)(IO) < (diostatus_t)0)

#define CHK_IO_SUCCESS(IO) \
    ((diostatus_t)(IO) > (diostatus_t)0)

#define CHK_IO_TIMEOUT(IO) \
    ((diostatus_t)(IO) == (diostatus_t)0)

#define CHK_IO_EMPTY(IO) CHK_IO_TIMEOUT(IO)

#define SERIAL_9600_BAUDRATE    9600
#define SERIAL_19200_BAUDRATE   19200
#define SERIAL_38400_BAUDRATE   38400
#define SERIAL_57600_BAUDRATE   57600
#define SERIAL_115200_BAUDRATE  115200

SERIAL serial_open(const char* dev, int baudrate);
void   serial_close(SERIAL);

diostatus_t serial_write(SERIAL port, const char* data, unsigned long size);
diostatus_t serial_write_byte(SERIAL port, const char data);
diostatus_t serial_read(SERIAL port, char* buf, unsigned long len);
diostatus_t serial_available(SERIAL port);
diostatus_t serial_available_for(SERIAL port, int microseconds);

#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/

#endif /* SERIAL_H */
