#ifndef SERIAL_H
#define SERIAL_H

#ifndef STATIC_ASSERT
# define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]
#endif

// TODO: GCC check
typedef unsigned char       duint8_t;
typedef unsigned short      duint16_t;
typedef unsigned int        duint32_t;
typedef unsigned long long  duint64_t;

typedef signed char         dint8_t;
typedef signed short        dint16_t;
typedef signed int          dint32_t;
typedef signed long long    dint64_t;

typedef int SERIAL;

#define SERIAL_9600_BAUDRATE 9600
#define SERIAL_19200_BAUDRATE 19200
#define SERIAL_38400_BAUDRATE 38400
#define SERIAL_57600_BAUDRATE 57600
#define SERIAL_115200_BAUDRATE 115200

SERIAL serial_open(const char* dev, int baudrate);
void   serial_close(SERIAL);

int serial_write(SERIAL port, const char* data, unsigned long size);
int serial_write(SERIAL port, const char data);
int serial_read(SERIAL port, char* buf, unsigned long len);
int serial_available(SERIAL port);
bool serial_available(SERIAL port, int microseconds);

#endif /* SERIAL_H */
