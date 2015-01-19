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

static struct MultiWiiIdent g_msp_data = {0};

#define MWI_POS_HEAD1   0
#define MWI_POS_HEAD2   1
#define MWI_POS_IO      2
#define MWI_POS_CMD     3
#define MWI_POS_DATALEN 4
#define MWI_POS_DATA    5

#define MWI_PACKET_SIZE(SZ) \
    SZ + 1 + MWI_POS_DATA

#define MWI_MASK        0xff

struct MultiWiiGetSetRaw {
    duint8_t len;
    duint8_t* data;
};

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
    if(!((g_hWii = fd) != -1))
        return False;

    if(CHK_IO_FAILED(multiwii_ident(&g_msp_data)))
        return False;

    return True;
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

diostatus_t multiwii_request(enum EMultiWiiCommand cmd)
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

dbool_t multiwii_exec(MULTIWII_CALLBACK func/* = 0*/,
                      void * data/* = 0*/,
                      dbool_t singleshot/* = False*/)
{
    if(g_hWii == -1)
        return False;

    duint8_t hash = 0;
    duint8_t cmdlen = 0;
    duint8_t cmdreaded = 0;
    duint8_t cmdid = 0;
    dbool_t endloop = False;

    while(!endloop) {
        /*
        if(g_state == MWS_IDLE)
            (void) multiwii_request(115);
*/

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
                /* exit from loop */
                if(singleshot)
                    endloop = True;
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
                    (void)func(mwc, data);

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

/*
 *
 * MultiWiiCommand struct
 *
 */

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

static void
multiwii_cmd_read(struct MultiWiiCommand* mwc, duint8_t *cb, duint8_t siz)
{
    while(siz--)
        *cb++ = multiwii_cmd_read8(mwc);
}

/*
 *
 * Real MSP function implements
 *
 */

static int
__msp_ident(struct MultiWiiCommand* cmd,
            struct MultiWiiIdent* data)
{
    data->version = multiwii_cmd_read8(cmd);
    data->type = multiwii_cmd_read8(cmd);
    data->protocol_version = multiwii_cmd_read8(cmd);
    data->cap = multiwii_cmd_read32(cmd);

    printf("$>IDENT(version: %hhu, type: %hhu, protocol version: %hhu, cap: 0x%08x)\n",
           data->version, data->type, data->protocol_version, data->cap);

    return 1;
}

static int
__msp_status(struct MultiWiiCommand* cmd,
             struct MultiWiiStatus* data)
{
    data->cycleTime = multiwii_cmd_read16(cmd);
    data->i2c_errors_count = multiwii_cmd_read16(cmd);
    data->sensor = multiwii_cmd_read16(cmd);
    data->flag = multiwii_cmd_read32(cmd);
    data->set = multiwii_cmd_read8(cmd);

    printf("$>STATUS(cycle time: %hu, i2c errors: %hu, sensor: %hu, flag: 0x%08x, set: %hhu)\n",
           data->cycleTime, data->i2c_errors_count, data->sensor, data->flag, data->set);

    return 1;
}

static int
__msp_raw_imu(struct MultiWiiCommand* cmd,
              struct MultiWiiImu* data)
{
    multiwii_cmd_read(cmd, (duint8_t*)data, sizeof(struct MultiWiiStatus));

    printf("$>RAW IMU(acc smooth: %02hx%02hx%02hx, gyro data: %02hx%02hx%02hx, mag ADC: %02hx%02hx%02hx)\n",
           data->accSmooth[0], data->accSmooth[1], data->accSmooth[2],
            data->gyroData[0], data->gyroData[1], data->gyroData[2],
            data->magADC[0], data->magADC[1], data->magADC[2]
            );

    return 1;
}

static int
__msp_get_raw(struct MultiWiiCommand* cmd,
              struct MultiWiiGetSetRaw* data)
{
    multiwii_cmd_read(cmd, data->data, data->len);

    return 1;
}

/*
 *
 *
 */

diostatus_t
multiwii_ident(struct MultiWiiIdent *ident)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_IDENT)))
        return eStatus;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_ident,
                      (void*)ident, True))
        return -1;

    return 0;
}

diostatus_t
multiwii_status(struct MultiWiiStatus* status)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_STATUS)))
        return eStatus;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_status,
                      (void*)status, True))
        return -1;

    return 0;
}

diostatus_t
multiwii_raw_imu(struct MultiWiiImu* imu)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_RAW_IMU)))
        return eStatus;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_raw_imu,
                      (void*)imu, True))
        return -1;

    return 0;
}

diostatus_t
multiwii_servo(duint16_t servo[8])
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_SERVO)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(servo);
    rawdata.data = (duint8_t*)servo;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>SERVO(%02hx %02hx %02hx %02hx %02hx %02hx %02hx %02hx)\n",
           servo[0], servo[1], servo[2], servo[3], servo[4], servo[5], servo[6],
            servo[7]
            );

    return 0;
}

diostatus_t
multiwii_motor(duint16_t motor[8])
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_MOTOR)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(motor);
    rawdata.data = (duint8_t*)motor;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>MOTOR(%02hx %02hx %02hx %02hx %02hx %02hx %02hx %02hx)\n",
           motor[0], motor[1], motor[2], motor[3], motor[4], motor[5], motor[6],
            motor[7]
            );

    return 0;
}

diostatus_t
multiwii_rc(duint16_t* rcData, duint8_t len)
{
    if(len != 18 && len != 12 && len != 8)
        return -1;

    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_RC)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = len * sizeof(duint16_t);
    rawdata.data = (duint8_t*)rcData;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>RC(");
    duint8_t i = 0;
    while(i != len) {
        printf("%02hx ", rcData[i]);
        ++i;
    }
    printf("\b)\n");

    return 0;
}

diostatus_t
multiwii_raw_gps(struct MiltiWiiRawGps *raw_gps)
{
    /* TODO: check GPS aviable */

    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_RAW_GPS)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MiltiWiiRawGps);
    rawdata.data = (duint8_t*)raw_gps;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>RAW_GPS(fix: %hhu, numSat: %hhu, coord LAT: 0x%08x, coord LON: 0x%08x,"
           "altitude: %hd, speed: %hu, ground course: %hu)\n",
           raw_gps->fix, raw_gps->numSat, raw_gps->coordLat, raw_gps->coordLon,
           raw_gps->altitude, raw_gps->speed, raw_gps->ground_course
           );

    return 0;
}

diostatus_t
multiwii_comp_gps(struct MultiWiiCompGps* comp_gps)
{
    /* TODO: check GPS aviable */

    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_COMP_GPS)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiCompGps);
    rawdata.data = (duint8_t*)comp_gps;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>COMP_GPS(distance to home: %hu, direction to home: %hd, update: 0x%02hhx)\n",
           comp_gps->distance_to_home, comp_gps->direction_to_home, comp_gps->update
           );

    return 0;
}

diostatus_t
multiwii_attitude(struct MultiWiiAttitude* att)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_ATTITUDE)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiAttitude);
    rawdata.data = (duint8_t*)att;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>ATTITUDE(angle[0]: %hu, angle[1]: %hu, heading: %hu)\n",
           att->angle[0], att->angle[1], att->heading
            );

    return 0;
}

diostatus_t
multiwii_altitude(struct MultiWiiAltitude* alt)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_ALTITUDE)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiAltitude);
    rawdata.data = (duint8_t*)alt;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>ALTITUDE(EstAlt: %d cm, vario: %hd cm/s)\n",
           alt->EstAlt, alt->vario
           );

    return 0;
}

diostatus_t
multiwii_analog(struct MultiWiiAnalog* alt)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_ANALOG)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiAnalog);
    rawdata.data = (duint8_t*)alt;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>ANALOG(vbat: %hhu, intPowerMeterSum: %hu, rssi: %hu, amperage: %hu)\n",
           alt->vbat, alt->intPowerMeterSum, alt->rssi, alt->amperage
           );

    return 0;
}

diostatus_t
multiwii_rc_tuning(struct MultiWiiRcConfig* rc_config)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_RC_TUNING)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiRcConfig);
    rawdata.data = (duint8_t*)rc_config;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>RC TUNING(rate: %hhd, expo: %hhd, roll pitch: %hhd, yaw: %hhd"
           "dynThrPID: %hhd, thrMid8: %hhd, thrExpo8: %hhd)\n",
           rc_config->rcRate8, rc_config->rcExpo8, rc_config->rollPitchRate,
           rc_config->yawRate, rc_config->dynThrPID, rc_config->thrMid8,
           rc_config->thrExpo8
           );

    return 0;
}

diostatus_t
multiwii_pid(struct MultiWiiPids* pids)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_PID)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiPids);
    rawdata.data = (duint8_t*)pids;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    printf("$>PID(");
    enum MultiWiiPidEnum i = MWE_PIDROLL;
    while(i != MWE_PIDITEMS) {
        printf("p%d{p: %hhu, i: %hhu, d: %hhu}, ",
               i, pids->pids[i].P8, pids->pids[i].I8, pids->pids[i].D8);
        ++i;
    }
    printf("\b\b\n");

    return 0;
}

diostatus_t
multiwii_misc(struct MultiWiiMisc* misc)
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_MISC)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(struct MultiWiiMisc);
    rawdata.data = (duint8_t*)misc;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    /* FIXME: debug log */
    printf("$>MISC()\n");

    return 0;
}

diostatus_t
multiwii_motor_pins(duint8_t motor_pins[8])
{
    diostatus_t eStatus = 0;
    if(CHK_IO_FAILED(eStatus = multiwii_request(MSP_MOTOR_PINS)))
        return eStatus;

    struct MultiWiiGetSetRaw rawdata;
    rawdata.len = sizeof(motor_pins);
    rawdata.data = (duint8_t*)motor_pins;

    if(!multiwii_exec((MULTIWII_CALLBACK)__msp_get_raw,
                      (void*)&rawdata, True))
        return -1;

    /* FIXME: debug log */
    printf("$>MOTOR PINS()\n");

    return 0;
}

#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/
