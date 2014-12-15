#include "multiwii.h"
#include <malloc.h> /* malloc */
#include <memory.h> /* memcpy */

/*#include <stddef.h>*/
/*#include <stdio.h>*/
/*#include <unistd.h>*/
/*#include <sys/time.h>*/
/*#include <time.h>*/

static SERIAL g_hWii = -1;
static duint8_t g_bufRead[0xff];
static duint8_t g_bufSize = 0;
static duint8_t g_pos = 0;
static duint8_t g_state = MWS_IDLE;

#define MWI_POS_HEAD1   0
#define MWI_POS_HEAD2   1
#define MWI_POS_IO      2
#define MWI_POS_CMD     3
#define MWI_POS_DATALEN 4
#define MWI_POS_DATA    5

#define MWI_PACKET_SIZE(SZ) \
    SZ + 1 + MWI_POS_DATA

#define MWI_MASK        0xff

inline void multiwii_cmd_debug(MultiWiiCommand* mwc)
{
    printf("(command: %d, len: %d", mwc->id, (mwc->data) ? mwc->size : -1);
    if(mwc->data) {
        printf(" data:");
        for(duint8_t i = 0; i < mwc->size; ++i) {
            if((i % 2) == 0 && i != 0)
                printf(" ");

            printf("%02x", mwc->data[i] & MWI_MASK);
        } /*for*/
    } /*if*/

    printf(")\n");
    (void) fflush(stdout);
}

inline void clear_bufRead()
{
    *g_bufRead = g_bufSize = g_pos = 0;
}

inline void create_head(char* buf, char io)
{
    buf[MWI_POS_HEAD1] = '$';
    buf[MWI_POS_HEAD2] = 'M';
    buf[MWI_POS_IO] = (io == 'i') ? '<' : ((io == 'o') ? '>' : '|');
}

inline duint32_t read32()
{
    duint32_t t = g_bufRead[g_pos++] & MWI_MASK;
    t += (g_bufRead[g_pos++] & MWI_MASK) << 8;
    t += (g_bufRead[g_pos++] & MWI_MASK) << 16;
    t += (g_bufRead[g_pos++] & MWI_MASK) << 24;
    return t;
}

inline duint16_t read16()
{
    duint16_t t = g_bufRead[g_pos++] & MWI_MASK;
    t += g_bufRead[g_pos++] << 8;
    return t;
}

inline duint8_t read8()
{
    return (g_bufRead[g_pos++] & MWI_MASK);
}

bool multiwii_init(SERIAL fd)
{
    return (g_hWii = fd) != -1;
}

duint8_t multiwii_state()
{
    return g_state;
}

inline bool multiwii_packet_check(duint8_t* data, duint16_t len)
{
    if(MWI_PACKET_SIZE(0) > len)
        return false;

    duint16_t pos = 0;

    if(data[pos++] != duint8_t('$'))
        return false;

    if(data[pos++] != duint8_t('M'))
        return false;

    if(data[pos] != duint8_t('|')
            || data[pos] != duint8_t('<')
            || data[pos] != duint8_t('>'))
        return false;

    ++pos;

    duint8_t storedLen = data[pos++];
    if(MWI_PACKET_SIZE(storedLen) != len)
        return false;

    duint8_t hash = 0;
    hash ^= data[pos]; /*len*/

    hash ^= data[pos++]; /*cmd*/
    for(duint8_t i = 0; i < storedLen; ++i) {
        hash ^= data[pos++];
    } // for

    if(data[pos] != hash)
        return false;

    return true;
}

bool multiwii_read(bool clear = true)
{
    if(clear)
        clear_bufRead();

    duint8_t buf = 0;

    while(serial_available(g_hWii)) {
        buf = 0;
        (void) serial_read(g_hWii, (char*)&buf, sizeof(buf));

        g_bufRead[g_bufSize++] = buf;
    } /*while*/

    if(duint8_t(-1) == g_bufSize) {
        g_bufSize = 0;
        return perror(__PRETTY_FUNCTION__), false;
    } else if(g_bufSize == 0)
        return false;
    else
        g_bufRead[g_bufSize] = 0;

    return true;
}

bool multiwii_write(duint8_t /*cmd*/, const duint8_t* /*data*/, duint16_t /*len*/)
{
    return false;
}

bool multiwii_write(duint8_t cmd)
{
    if(multiwii_ready())
        return false;

    (void) serial_write(g_hWii, '$');
    serial_write(g_hWii, 'M');
    serial_write(g_hWii, '<');
    serial_write(g_hWii, 0);
    serial_write(g_hWii, cmd);
    serial_write(g_hWii, cmd);

    return true;
}

bool multiwii_exec(MULTIWII_CALLBACK func/* = 0*/)
{
    (void) func;

    if(g_hWii == -1)
        return false;

    duint8_t hash = 0;
    duint8_t cmdlen = 0;
    duint8_t cmdreaded = 0;
    duint8_t cmdid = 0;

    printf("Hello, World!");
    while(1) {
        if(g_state == MWS_IDLE)
            multiwii_write(115);
        if(!serial_available(g_hWii, 5000)) {
            // do error
            continue;
        }

        if(g_state == MWS_IDLE)
            clear_bufRead();

        duint8_t buf = 0;

        while(serial_available(g_hWii)) {
            buf = 0;
            if(1 != serial_read(g_hWii, (char*)&buf, sizeof(buf)))
                continue;

            switch(g_state) {
            case MWS_IDLE:
                hash = cmdlen = cmdid = cmdreaded = 0;
                if (buf == '$')
                    g_state = MWS_HEAD_START;
                break;
            case MWS_HEAD_START:
                if(buf == 'M')
                    g_state = MWS_HEAD_M;
                break;
            case MWS_HEAD_M:
                if(buf == '>')
                    g_state = MWS_HEAD_ARROW;
                break;
            case MWS_HEAD_ARROW:
                cmdlen = hash = buf;
                g_state = MWS_HEAD_SIZE;
                break;
            case MWS_HEAD_SIZE:
                cmdid = buf;
                hash ^= buf;
                g_state = MWS_HEAD_CMD;
                break;
            case MWS_HEAD_CMD:
                cmdreaded++;
                g_bufRead[g_bufSize++] = buf;
                hash ^= buf;

                if(cmdlen <= cmdreaded)
                    g_state = MWS_HEAD_HASH;
                break;
            case MWS_HEAD_HASH:
                if(buf != hash)
                    printf("(error)\n");

            {
                /* TODO: process packet */
                MultiWiiCommand* mwc = multiwii_cmd_init(cmdid, g_bufRead,
                                                         g_bufSize);
                /* multiwii_cmd_debug(mwc); */
                if(func)
                    (void)func(mwc);

                multiwii_cmd_free(mwc);
            }

                // done
                g_state = MWS_IDLE;
                break;
            default:
                /* DEBUGBREAK; */
                ;
            }
        } /* while read */

    } /* while events loop */

    return true;
}

MultiWiiCommand* multiwii_cmd_init(duint8_t cmdid, const duint8_t* data/* = 0*/,
                                   duint8_t len/* = 0*/)
{
    MultiWiiCommand* cmd = (MultiWiiCommand*)malloc(sizeof(MultiWiiCommand));
    if(cmd == 0)
        return (MultiWiiCommand*)0;

    cmd->id = cmdid;
    if(data == 0) {
        cmd->data = 0;
        cmd->size = 0;
    } else {
        cmd->data = (duint8_t*)malloc(sizeof(duint8_t) * len);
        if(cmd->data == 0)
            return (void)free((void*)cmd), (MultiWiiCommand*)0;

        cmd->size = len;
        (void)memcpy(cmd->data, data, len);
    }

    cmd->is_filled = true;
    cmd->pos = 0;

    return cmd;
}

void multiwii_cmd_free(MultiWiiCommand* mwc)
{
    if(mwc->data)
        (void)free(mwc->data), mwc->data = 0;

    (void) free((void*)mwc);
}

duint8_t multiwii_cmd_read8(MultiWiiCommand* mwc)
{
    return (mwc->data[mwc->pos++] & MWI_MASK);
}

duint16_t multiwii_cmd_read16(MultiWiiCommand* mwc)
{
    duint16_t t = mwc->data[mwc->pos++] & MWI_MASK;
    t += mwc->data[mwc->pos++] << 8;

    return t;
}

duint32_t multiwii_cmd_read32(MultiWiiCommand* mwc)
{
    duint32_t t = mwc->data[mwc->pos++] & MWI_MASK;
    t += (mwc->data[mwc->pos++] & MWI_MASK) << 8;
    t += (mwc->data[mwc->pos++] & MWI_MASK) << 16;
    t += (mwc->data[mwc->pos++] & MWI_MASK) << 24;

    return t;
}
