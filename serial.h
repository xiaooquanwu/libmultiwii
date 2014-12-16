#ifndef SERIAL_H
#define SERIAL_H

#include "dlib.hxx"

#ifdef __cplusplus
extern "C" {
#endif /*cpp*/

typedef int SERIAL;

#define SERIAL_9600_BAUDRATE    9600
#define SERIAL_19200_BAUDRATE   19200
#define SERIAL_38400_BAUDRATE   38400
#define SERIAL_57600_BAUDRATE   57600
#define SERIAL_115200_BAUDRATE  115200

SERIAL serial_open(const char* dev, int baudrate);
void   serial_close(SERIAL);

int serial_write(SERIAL port, const char* data, unsigned long size);
int serial_write_byte(SERIAL port, const char data);
int serial_read(SERIAL port, char* buf, unsigned long len);
int serial_available(SERIAL port);
dbool_t serial_available_for(SERIAL port, int microseconds);

#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/

#endif /* SERIAL_H */
