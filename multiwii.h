#ifndef MULTIWII_H
#define MULTIWII_H

#ifdef __cplusplus
extern "C" {
#endif /*cpp*/

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

typedef int (*MULTIWII_CALLBACK)(struct MultiWiiCommand*);

dbool_t multiwii_init(SERIAL);

duint8_t multiwii_state();

dbool_t multiwii_ready();

dbool_t multiwii_read(dbool_t clear);

dbool_t multiwii_send(duint8_t cmd, const duint8_t* data, duint16_t len);

dbool_t multiwii_request(duint8_t cmd);

void multiwii_debug_read(dbool_t read);

dbool_t multiwii_exec(MULTIWII_CALLBACK func/* = 0*/);

struct MultiWiiCommand* multiwii_cmd_init(duint8_t cmd, const duint8_t* data/* = 0*/,
                                          duint8_t len/* = 0*/);

void multiwii_cmd_free(struct MultiWiiCommand*);
duint8_t multiwii_cmd_read8(struct MultiWiiCommand*);
duint16_t multiwii_cmd_read16(struct MultiWiiCommand*);
duint32_t multiwii_cmd_read32(struct MultiWiiCommand*);

#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/

#endif // MULTIWII_H
