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

enum EMultiWiiCommand {
    MSP_PRIVATE            = 1,    /* in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet */

    MSP_IDENT              = 100,  /* out message         multitype + multiwii version + protocol version + capability variable */
    MSP_STATUS             = 101,  /* out message         cycletime & errors_count & sensor present & box activation & current setting number */
    MSP_RAW_IMU            = 102,  /* out message         9 DOF */
    MSP_SERVO              = 103,  /* out message         8 servos */
    MSP_MOTOR              = 104,  /* out message         8 motors */
    MSP_RC                 = 105,  /* out message         8 rc chan and more */
    MSP_RAW_GPS            = 106,  /* out message         fix, numsat, lat, lon, alt, speed, ground course */
    MSP_COMP_GPS           = 107,  /* out message         distance home, direction home */
    MSP_ATTITUDE           = 108,  /* out message         2 angles 1 heading */
    MSP_ALTITUDE           = 109,  /* out message         altitude, variometer */
    MSP_ANALOG             = 110,  /* out message         vbat, powermetersum, rssi if available on RX */
    MSP_RC_TUNING          = 111,  /* out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID */
    MSP_PID                = 112,  /* out message         PID coeff (9 are used currently) */
    MSP_BOX                = 113,  /* out message         BOX setup (number is dependant of your setup) */
    MSP_MISC               = 114,  /* out message         powermeter trig */
    MSP_MOTOR_PINS         = 115,  /* out message         which pins are in use for motors & servos, for GUI */
    MSP_BOXNAMES           = 116,  /* out message         the aux switch names */
    MSP_PIDNAMES           = 117,  /* out message         the PID names */
    MSP_WP                 = 118,  /* out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold */
    MSP_BOXIDS             = 119,  /* out message         get the permanent IDs associated to BOXes */
    MSP_SERVO_CONF         = 120,  /* out message         Servo settings */

    MSP_NAV_STATUS         = 121,  /* out message         Returns navigation status */
    MSP_NAV_CONFIG         = 122,  /* out message         Returns navigation parameters */

    MSP_CELLS              = 130,  /* out message         FRSKY Battery Cell Voltages */

    MSP_SET_RAW_RC         = 200,  /* in message          8 rc chan */
    MSP_SET_RAW_GPS        = 201,  /* in message          fix, numsat, lat, lon, alt, speed */
    MSP_SET_PID            = 202,  /* in message          P I D coeff (9 are used currently) */
    MSP_SET_BOX            = 203,  /* in message          BOX setup (number is dependant of your setup) */
    MSP_SET_RC_TUNING      = 204,  /* in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID */
    MSP_ACC_CALIBRATION    = 205,  /* in message          no param */
    MSP_MAG_CALIBRATION    = 206,  /* in message          no param */
    MSP_SET_MISC           = 207,  /* in message          powermeter trig + 8 free for future use */
    MSP_RESET_CONF         = 208,  /* in message          no param */
    MSP_SET_WP             = 209,  /* in message          sets a given WP (WP#,lat, lon, alt, flags) */
    MSP_SELECT_SETTING     = 210,  /* in message          Select Setting Number (0-2) */
    MSP_SET_HEAD           = 211,  /* in message          define a new heading hold direction */
    MSP_SET_SERVO_CONF     = 212,  /* in message          Servo settings */
    MSP_SET_MOTOR          = 214,  /* in message          PropBalance function */
    MSP_SET_NAV_CONFIG     = 215,  /* in message          Sets nav config parameters - write to the eeprom */

    MSP_SET_ACC_TRIM       = 239,  /* in message          set acc angle trim values */
    MSP_ACC_TRIM           = 240,  /* out message         get acc angle trim values */
    MSP_BIND               = 241,  /* in message          no param */

    MSP_EEPROM_WRITE       = 250,  /* in message          no param */

    MSP_DEBUGMSG           = 253,  /* out message         debug string buffer */
    MSP_DEBUG              = 254,  /* out message         debug1,debug2,debug3,debug4 */
};

typedef int (*MULTIWII_CALLBACK)(struct MultiWiiCommand*, void* data);

enum MultiWiiRcEnum {
    MWE_ROLL,
    MWE_PITCH,
    MWE_YAW,
    MWE_THROTTLE,
    MWE_AUX1,
    MWE_AUX2,
    MWE_AUX3,
    MWE_AUX4,
    MWE_AUX5,
    MWE_AUX6,
    MWE_AUX7,
    MWE_AUX8
};

enum MultiWiiPidEnum {
    MWE_PIDROLL,
    MWE_PIDPITCH,
    MWE_PIDYAW,
    MWE_PIDALT,
    MWE_PIDPOS,
    MWE_PIDPOSR,
    MWE_PIDNAVR,
    MWE_PIDLEVEL,
    MWE_PIDMAG,
    MWE_PIDVEL,     /* legacy, not used currently */
    MWE_PIDITEMS
};


dbool_t
multiwii_init(SERIAL);

enum MultiWiiState
        multiwii_state();

dbool_t
multiwii_ready();

diostatus_t
multiwii_read(dbool_t clear/* = true*/);

diostatus_t
multiwii_send(duint8_t cmd, const duint8_t* data, duint16_t len);

diostatus_t
multiwii_request(enum EMultiWiiCommand cmd);

dbool_t
multiwii_exec(MULTIWII_CALLBACK func/* = 0*/,
              void * data/* = 0*/,
              dbool_t sinleshot /*= False*/);

struct MultiWiiCommand*
        multiwii_cmd_init(duint8_t cmd, const duint8_t* data/* = 0*/,
                          duint8_t len/* = 0*/);

void
multiwii_cmd_free(struct MultiWiiCommand*);

duint8_t
multiwii_cmd_read8(struct MultiWiiCommand*);

duint16_t
multiwii_cmd_read16(struct MultiWiiCommand*);

duint32_t
multiwii_cmd_read32(struct MultiWiiCommand*);

/*
 *
 *
 *
 */

struct MultiWiiIdent {
    duint8_t  version;
    duint8_t  type;
    duint8_t  protocol_version;
    duint32_t cap;
};

diostatus_t
multiwii_ident(struct MultiWiiIdent* ident);

struct MultiWiiStatus {
    duint16_t cycleTime;
    duint16_t i2c_errors_count;
    duint16_t sensor;
    duint32_t flag;
    duint8_t set;
};

diostatus_t
multiwii_status(struct MultiWiiStatus* status);

struct MultiWiiImu {
    dint16_t  accSmooth[3];
    dint16_t  gyroData[3];
    dint16_t  magADC[3];
    /*  dint16_t  gyroADC[3]; */
    /*  dint16_t  accADC[3];  */
};

diostatus_t
multiwii_raw_imu(struct MultiWiiImu* imu);

diostatus_t
multiwii_servo(duint16_t servo[8]);

diostatus_t
multiwii_motor(duint16_t motor[8]);

diostatus_t
multiwii_rc(duint16_t* rcData, duint8_t len);

struct MiltiWiiRawGps {
    duint8_t  fix;
    duint8_t  numSat;
    dint32_t  coordLat;
    dint32_t  coordLon;
    dint16_t  altitude;
    duint16_t speed;
    duint16_t ground_course;
};

diostatus_t
multiwii_raw_gps(struct MiltiWiiRawGps* raw_gps);

struct MultiWiiCompGps {
    duint16_t distance_to_home;
    dint16_t  direction_to_home;
    duint8_t  update;
};

diostatus_t
multiwii_comp_gps(struct MultiWiiCompGps* comp_gps);

struct MultiWiiAttitude {
    dint16_t angle[2];    /* absolute angle inclination in multiple of 0.1 degree    180 deg = 1800 */
    dint16_t heading;     /* variometer in cm/s */
};

diostatus_t
multiwii_attitude(struct MultiWiiAttitude* att);

struct MultiWiiAltitude {
    dint32_t  EstAlt;      /* in cm */
    dint16_t  vario;       /* variometer in cm/s */
};

diostatus_t
multiwii_altitude(struct MultiWiiAltitude* alt);

struct MultiWiiAnalog {
    duint8_t  vbat;              /* battery voltage in 0.1V steps */
    duint16_t intPowerMeterSum;
    duint16_t rssi;              /* range: [0;1023] */
    duint16_t amperage;          /* 1unit == 100mA */

    /*  duint16_t watts;             // 1unit == 1W */
    /*  duint16_t vbatcells[VBAT_CELLS_NUM]; */
};

diostatus_t
multiwii_analog(struct MultiWiiAnalog* alt);

struct MultiWiiRcConfig {
    duint8_t rcRate8;
    duint8_t rcExpo8;
    duint8_t rollPitchRate;
    duint8_t yawRate;
    duint8_t dynThrPID;
    duint8_t thrMid8;
    duint8_t thrExpo8;
};

diostatus_t
multiwii_rc_tuning(struct MultiWiiRcConfig* rc_config);

struct MultiWiiPid {
    duint8_t P8;
    duint8_t I8;
    duint8_t D8;
};

struct MultiWiiPids {
    struct MultiWiiPid pids[MWE_PIDITEMS];
};

diostatus_t
multiwii_pid(struct MultiWiiPids* pids);

struct MultiWiiMisc {
    duint16_t intPowerTrigger1;
    duint16_t throttle_min;
    duint16_t throttle_max;
    duint16_t command_min;
    duint16_t failsafe_throttle;    /* if define FAILSAFE */
    duint16_t log_arm;              /* if define LOG_PERMANENT */
    duint32_t log_time;             /* if define LOG_PERMANENT */
    duint16_t mag_declination;      /* if define MAG */
    /* if define VBAT */
    duint8_t vbatscale;
    duint8_t vbatlevel_warn1;
    duint8_t vbatlevel_warn2;
    duint8_t vbatlevel_crit;
};

diostatus_t
multiwii_misc(struct MultiWiiMisc* misc);

diostatus_t
multiwii_motor_pins(duint8_t motor_pins[8]);

/*
9    MSP_BOX
8    MSP_BOXNAMES
7    MSP_PIDNAMES
6    MSP_WP
5    MSP_BOXIDS
4    MSP_SERVO_CONF
3    MSP_NAV_STATUS
2    MSP_NAV_CONFIG
1    MSP_CELLS
*/


#ifdef __cplusplus
} /* extern C */
#endif /*cpp*/

#endif /* MULTIWII_H */
