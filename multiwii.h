#ifndef MULTIWII_H
#define MULTIWII_H

#ifdef __cplusplus
extern "C" {
#endif /*cpp*/

#include "serial.h"

enum MultiWiiState {
    MWS_IDLE,
    MWS_WAIT_NEXT,      /* same as MWS_IDLE, but reading data is aviable */

    MWS_HEAD_START,
    MWS_HEAD_M,
    MWS_HEAD_ARROW,
    MWS_HEAD_SIZE,
    MWS_HEAD_CMD,
    MWS_HEAD_DATA,
    MWS_HEAD_HASH
};

struct MultiWiiCommand {
    duint8_t id;        /* command id */
    dbool_t  is_filled; /* data is ready */
    duint8_t size;      /* size of data */
    duint8_t *data;     /* if null then empty */
    duint8_t pos;       /* readed data */
};

typedef int (*MULTIWII_CALLBACK)(struct MultiWiiCommand*);

dbool_t multiwii_init(SERIAL);

enum MultiWiiState multiwii_state();

dbool_t multiwii_ready();

diostatus_t multiwii_read(dbool_t clear/* = true*/);

diostatus_t multiwii_send(duint8_t cmd, const duint8_t* data, duint16_t len);

diostatus_t multiwii_request(duint8_t cmd);

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

#endif /* MULTIWII_H */
