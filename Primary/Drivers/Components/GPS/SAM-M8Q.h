#ifndef SAM_M8Q_H_
#define SAM_M8Q_H_

extern I2C_HandleTypeDef hi2c2;

#define GPS_I2C        hi2c2 
#define GPS_I2C_ADDR  (0x42 << 1) 

#define GPS_MESSAGE_CLASS_CFG 0x06

#define GPS_MESSAGE_ID_0 0x00

#define GPS_PORT_ID 0x00

typedef struct
{
    int32_t messageClass;       

    int32_t messageId;

    char* messageBody;

    uint8_t status;

} UBX_MessageType; //On failure to extract message all fields 0/NULL

#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;     // GPS time of week in milliseconds
    int32_t lon;       // Longitude in degrees * 1e-7
    int32_t lat;       // Latitude in degrees * 1e-7
    int32_t height;    // Height above ellipsoid in millimeters
    int32_t hMSL;      // Height above mean sea level in millimeters
    uint32_t hAcc;     // Horizontal accuracy estimate in millimeters
    uint32_t vAcc;     // Vertical accuracy estimate in millimeters
} ubx_nav_posllh_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;     // GPS time of week in milliseconds
    uint16_t year;     // Year (UTC)
    uint8_t month;     // Month (UTC) 1-12
    uint8_t day;       // Day of month (UTC) 1-31
    uint8_t hour;      // Hour of day (UTC) 0-23
    uint8_t min;       // Minute of hour (UTC) 0-59
    uint8_t sec;       // Second of minute (UTC) 0-59
    uint8_t valid;     // Validity flags (see below)
    uint32_t tAcc;     // Time accuracy estimate in nanoseconds
    int32_t nano;      // Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t gpsFix;    // GPS fix status (0: no fix, 1: Dead Reckoning only, 2: 2D fix, 3: 3D fix, 4:GPS + dead reckoning, 5: Time only fix)
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;     // Number of satellites used in Nav Solution
    int32_t lon;       // Longitude in degrees * 1e-7
    int32_t lat;       // Latitude in degrees * 1e-7
    int32_t height;    // Height above ellipsoid in millimeters
    int32_t hMSL;      // Height above mean sea level in millimeters
    uint32_t hAcc;     // Horizontal accuracy estimate in millimeters
    uint32_t vAcc;     // Vertical accuracy estimate in millimeters
    int32_t velN;      // Velocity north in millimeters/second
    int32_t velE;      // Velocity east in millimeters/second
    int32_t velD;      // Velocity down in millimeters/second
    int32_t gSpeed;    // 2D Ground speed in millimeters/second
    int32_t headMot;   // Heading of motion 2D in degrees * 1e-5
    int32_t sAcc;      // Speed accuracy estimate in millimeters/second
    int32_t headAcc;   // Heading accuracy estimate in degrees * 1e-5
    uint16_t pDOP;     // Position DOP (* 0.01)
    uint16_t flags3;   // Flags
    uint32_t reserved1;// Reserved for future use
    uint32_t headVeh;   // Heading of vehicle in degrees
    uint16_t magDec;   // Magnetic declination in degrees (not used)
    uint16_t magAcc;   // Magnetic declination accuracy in degrees (not used)
} UBX_NAV_PVT;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t measRate;  // Measurement rate in milliseconds (1000 for 1Hz, 100 for 10Hz, etc.)
    uint16_t navRate;   // he ratio between the number of measurements and the number of navigation solutions, e.g. 5 means five measurements for every navigation solution.
    uint16_t timeRef;   // Time reference (0 for GPS time, 1 for UTC time)
} UBX_CFG_RATE;
#pragma pack(pop)

extern UBX_NAV_PVT gps_data;

uint8_t GPS_VER_CHECK(void);

void GPS_Init(void);

UBX_MessageType ublox_ReadOutput(char* UBX_MessageReturn);

uint8_t GPS_ReadSensorData(UBX_NAV_PVT *posllh);
uint8_t GPS_RequestSensorData(void);
uint8_t GPS_ReadNavPVT(UBX_NAV_PVT *posllh);

#endif /* SAM_M8Q_H_ */