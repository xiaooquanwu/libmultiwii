#include "multiwii.h"
#include <malloc.h> /* malloc */
#include <memory.h> /* memcpy */

#ifdef __cplusplus
extern "C" {
#endif /*cpp*/

static SERIAL g_hWii = -1;
static duint8_t g_bufRead[0xff];
static duint8_t g_bufSize = 0;
static duint8_t g_pos = 0;
static enum MultiWiiState g_state = MWS_IDLE;

#define MWI_POS_HEAD1   0
#define MWI_POS_HEAD2   1
#define MWI_POS_IO      2
#define MWI_POS_CMD     3
#define MWI_POS_DATALEN 4
#define MWI_POS_DATA    5

#define MWI_PACKET_SIZE(SZ) \
    SZ + 1 + MWI_POS_DATA

#define MWI_MASK        0xff

/*inline*/ void multiwii_cmd_debug(struct MultiWiiCommand* mwc)
{
    dint8_t i = 0;

    printf("(command: %d, len: %d", mwc->id, (mwc->data) ? mwc->size : -1);
    if(mwc->data) {
        printf(" data:");
        while(i < mwc->size) {
            if((i % 2) == 0 && i != 0)
                printf(" ");

            printf("%02x", mwc->data[i] & MWI_MASK);

            ++i;
        } /*for*/
    } /*if*/

    printf(")\n");
    (void) fflush(stdout);
}

/*inline*/ void clear_bufRead()
{
    *g_bufRead = g_bufSize = g_pos = 0;
}

dbool_t multiwii_init(SERIAL fd)
{
    return (g_hWii = fd) != -1;
}

enum MultiWiiState multiwii_state()
{
    return g_state;
}

dbool_t multiwii_ready()
{
    return multiwii_state() == MWS_IDLE;
}

/*inline*/ dbool_t multiwii_packet_check(duint8_t* data, duint16_t len)
{
    if(MWI_PACKET_SIZE(0) > len)
        return False;

    duint16_t pos = 0;

    if(data[pos++] != (duint8_t)'$')
        return False;

    if(data[pos++] != (duint8_t)'M')
        return False;

    if(data[pos] != (duint8_t)'|'
            || data[pos] != (duint8_t)'<'
            || data[pos] != (duint8_t)'>')
        return False;

    ++pos;

    duint8_t storedLen = data[pos++];
    if(MWI_PACKET_SIZE(storedLen) != len)
        return False;

    duint8_t hash = 0;
    hash ^= data[pos]; /*len*/

    hash ^= data[pos++]; /*cmd*/
    duint8_t i = 0;
    while(i < storedLen) {
        hash ^= data[pos++];
        ++i;
    } /*while*/

    if(data[pos] != hash)
        return False;

    return True;
}

diostatus_t multiwii_read(dbool_t clear/* = true*/)
{
    if(clear)
        clear_bufRead();

    duint8_t buf = 0;

    while(serial_available(g_hWii)) {
        buf = 0;
        if(!CHK_IO_SUCCESS(serial_read(g_hWii, (char*)&buf, sizeof(buf))))
            return -1;

        g_bufRead[g_bufSize++] = buf;
    } /*while*/

    if((duint8_t)-1 == (duint8_t)g_bufSize) {
        /*FIXME*/
        g_bufSize = 0;
        return perror(__PRETTY_FUNCTION__), -1;
    } else if(g_bufSize == 0)
        return D_IO_EMPTY;
    else
        g_bufRead[g_bufSize] = 0;

    return g_bufSize;
}

diostatus_t multiwii_send(duint8_t cmd, const duint8_t* data, duint16_t len)
{
    D_UNUSED(cmd); D_UNUSED(data); D_UNUSED(len);

    return D_IO_EMPTY;
}

diostatus_t multiwii_request(duint8_t cmd)
{
    if(!multiwii_ready())
        return D_IO_EMPTY;

    if(CHK_IO_SUCCESS(serial_write_byte(g_hWii, '$'))
            && CHK_IO_SUCCESS(serial_write_byte(g_hWii, 'M'))
            && CHK_IO_SUCCESS(serial_write_byte(g_hWii, '<'))
            && CHK_IO_SUCCESS(serial_write_byte(g_hWii, 0))
            && CHK_IO_SUCCESS(serial_write_byte(g_hWii, cmd))
            && CHK_IO_SUCCESS(serial_write_byte(g_hWii, cmd)))
        return 6;

    /* FIXME: set correct errno */
    return -1;
}

dbool_t multiwii_exec(MULTIWII_CALLBACK func/* = 0*/)
{
    (void) func;

    if(g_hWii == -1)
        return False;

    duint8_t hash = 0;
    duint8_t cmdlen = 0;
    duint8_t cmdreaded = 0;
    duint8_t cmdid = 0;

    while(1) {
/**/
        if(g_state == MWS_IDLE)
            (void) multiwii_request(115);
/**/

        if(!CHK_IO_SUCCESS(serial_available_for(g_hWii, 5000))) {
            /* do error */
            continue;
        }

        if(g_state == MWS_IDLE)
            clear_bufRead();

        duint8_t buf = 0;

        while(CHK_IO_SUCCESS(serial_available(g_hWii))) {
            buf = 0;
            if(!CHK_IO_SUCCESS(serial_read(g_hWii, (char*)&buf, sizeof(buf))))
                continue;

            switch(g_state) {
            case MWS_IDLE:
            case MWS_WAIT_NEXT:
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

                g_state = CHK_IO_SUCCESS(serial_available(g_hWii)) ? MWS_WAIT_NEXT: MWS_IDLE;
            {
                /* TODO: process packet */
                struct MultiWiiCommand* mwc = multiwii_cmd_init(cmdid,
                                                                g_bufRead,
                                                                g_bufSize);
                /*TODO: die_mem*/

                /**/
                multiwii_cmd_debug(mwc);
                /**/
                if(func)
                    (void)func(mwc);

                multiwii_cmd_free(mwc);
            }

                /* done */
                break;
            default:
                /* DEBUGBREAK; */
                ;
            }
        } /* while read */

    } /* while events loop */

    return True;
}

struct MultiWiiCommand* multiwii_cmd_init(duint8_t cmdid,
                                          const duint8_t* data/* = 0*/,
                                          duint8_t len/* = 0*/)
{
    struct MultiWiiCommand* cmd = (struct MultiWiiCommand*)malloc(sizeof(struct MultiWiiCommand));
    if(cmd == 0)
        return (struct MultiWiiCommand*)0;

    cmd->id = cmdid;
    if(data == 0) {
        cmd->data = 0;
        cmd->size = 0;
    } else {
        cmd->data = (duint8_t*)malloc(sizeof(duint8_t) * len);
        if(cmd->data == 0)
            return (void)free((void*)cmd), (struct MultiWiiCommand*)0;

        cmd->size = len;
        (void)memcpy(cmd->data, data, len);
    }

    cmd->is_filled = True;
    cmd->pos = 0;

    return cmd;
}

void multiwii_cmd_free(struct MultiWiiCommand* mwc)
{
    if(mwc->data)
        (void)free(mwc->data), mwc->data = 0;

    (void) free((void*)mwc);
}

duint8_t multiwii_cmd_read8(struct MultiWiiCommand* mwc)
{
    return (mwc->data[mwc->pos++] & MWI_MASK);
}

duint16_t multiwii_cmd_read16(struct MultiWiiCommand* mwc)
{
    duint16_t t = mwc->data[mwc->pos++] & MWI_MASK;
    t += mwc->data[mwc->pos++] << 8;

    return t;
}

duint32_t multiwii_cmd_read32(struct MultiWiiCommand* mwc)
{
    duint32_t t = mwc->data[mwc->pos++] & MWI_MASK;
    t += (mwc->data[mwc->pos++] & MWI_MASK) << 8;
    t += (mwc->data[mwc->pos++] & MWI_MASK) << 16;
    t += (mwc->data[mwc->pos++] & MWI_MASK) << 24;

    return t;
}

#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/
