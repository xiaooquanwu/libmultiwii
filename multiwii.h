#ifndef MULTIWII_H
#define MULTIWII_H

#include "serial.h"

#define MWS_IDLE        0
#define MWS_HEAD_START  1
#define MWS_HEAD_M      2
#define MWS_HEAD_ARROW  3
#define MWS_HEAD_SIZE   4
#define MWS_HEAD_CMD    5
#define MWS_HEAD_DATA   6
#define MWS_HEAD_HASH   7

struct MultiWiiCommand {
    duint8_t id;        /* command id */
    duint8_t is_filled; /* data is ready */
    duint8_t size;      /* size of data */
    duint8_t* data;     /* if null then empty */
    duint8_t pos;       /* readed data */
};

typedef int (*MULTIWII_CALLBACK)(MultiWiiCommand*);

bool multiwii_init(SERIAL);

duint8_t multiwii_state();

inline bool multiwii_ready()
{ return multiwii_state() == MWS_IDLE; }

bool multiwii_read(bool clear);

bool multiwii_write(char cmd, const duint8_t* data, duint16_t len);

bool multiwii_write(duint8_t cmd);

void multiwii_debug_read(bool read);

bool multiwii_exec(MULTIWII_CALLBACK func = 0);

//
//
//
MultiWiiCommand* multiwii_cmd_init(duint8_t cmd, const duint8_t* data = 0,
                                   duint8_t len = 0);

void multiwii_cmd_free(MultiWiiCommand*);
duint8_t multiwii_cmd_read8(MultiWiiCommand*);
duint16_t multiwii_cmd_read16(MultiWiiCommand*);
duint32_t multiwii_cmd_read32(MultiWiiCommand*);

#endif // MULTIWII_H
