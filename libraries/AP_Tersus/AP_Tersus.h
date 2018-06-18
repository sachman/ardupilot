
#pragma once

#include <AP_HAL/AP_HAL.h>
/*#include <../ArduCopter/Copter.h>*/
#include <GCS_MAVLink/GCS_MAVLink.h>

#define CHAR_TO_DIGIT(c)    (c - '0')

#define HEADER_SIZE 28 // Binary header size for OEM 4, V, and 6 receivers
#define CHECKSUM_SIZE 4  // size of the message CRC

#define SYNC_1_INDEX 0  // first sync byte location
#define SYNC_2_INDEX 1  // second sync byte location
#define SYNC_3_INDEX 2  // third sync byte location
#define HEADER_LEN_INDEX 3 // header length location
#define MSG_ID_END_INDEX 5  // Message ID location
#define MSG_LENGTH_END_INDEX 9 // message length index

#define TERSUS_SYNC_BYTE_1 0xAA
#define TERSUS_SYNC_BYTE_2 0x44
#define TERSUS_SYNC_BYTE_3 0x12
#define TERSUS_ACK_BYTE_1 '<'
#define TERSUS_ACK_BYTE_2 'O'
#define TERSUS_ACK_BYTE_3 'K'
//#define COMNAV_RESET_BYTE_1 0X5B
//#define COMNAV_RESET_BYTE_2 'C'
//#define COMNAV_RESET_BYTE_3 'O'
//#define COMNAV_RESET_BYTE_4 'M'

#define TERSUS_HEADING_MSG_ID 971   // Heading Msg information with the ALIGN feature.


#define TERSUS_POS_FIXED 50
#define TERSUS_POS_FLOAT 34

#define TERSUS_HEADING_LOG_FILE "/fs/microsd/tersusHeading_logFile.txt"

// extern void read_tersus_serial();
extern uint32_t tersus_serial_data;
/*extern float tersus_heading;*/
//extern Copter copter;



enum ascii_state {
    START_ASCII, GET_ASCII_HEADER, GET_ASCII_DATA, GET_ASCII_CRC, END_ASCII
};

enum nmea_state{
    START_NMEA, GET_NMEA_DATA, GET_NMEA_CRC, END_DATA
};

enum time_status_enum{
    UNKNOWN = 20, APPROXIMATE = 60, COARSEADJUSTING = 80, COARSE = 100, COARSESTEERING = 120, FREEWHEELING = 130, FINEADJUSTING = 140, FINE = 160,
    FINEBACKUPSTEERING = 170, FINESTEERING = 180, SATTIME = 200
};

enum solution_status{
    SOL_COMPUTED = 0, INSUFFICIENT_OBS = 1, NO_CONVERGENCE = 2, SINGULARITY = 3, COV_TRACE = 4, TEST_DIST = 5, COLD_START = 6, V_H_LIMIT = 7,
    VARIANCE = 8, RESIDUALS = 9, INTEGRITY_WARNING = 13, PENDING = 18, MAKE_ENUM_32BIT_1 = 0xFFFFFFFF
};

// Propagated enum value is chosen randomly since datasheet had the value omitted.
enum position_type{
    NONE = 0, FIXEDPOS = 1, FIXEDHEIGHT = 2, DOPPLER_VELOCITY = 8, SINGLE = 16, PSRDIFF = 17, WAAS = 18, PROPAGATED = 19, NARROW_FLOAT = 34,
    L1_INT = 48, NARROW_INT = 50, MAKE_ENUM_32BIT_2 = 0xFFFFFFFF
};

enum tersus_state{
    WAIT_TERSUS_SYNC1, WAIT_TERSUS_SYNC2, WAIT_TERSUS_SYNC3, GET_TERSUS_HEADER_INFO, GET_TERSUS_DATA, GET_TERSUS_CRC
};

struct __attribute__((__packed__)) tersus_Header{

    int8_t  sync1;                  //!< start of packet first byte (0xAA)
    int8_t  sync2;                  //!< start of packet second byte (0x44)
    int8_t  sync3;                  //!< start of packet third  byte (0x12)
    uint8_t hdr_len;                //!< Length of the header in bytes ( From start of packet )
    uint16_t msg_id;                //!< Message ID number
    int8_t msg_type;                //!< Message type - binary, ascii, nmea, etc...Reserved to 0x02
    uint8_t port_addr;              //!< Address of the data port the log was received on. COM1:32 COM2:33 USB:1440 FILE:8002
    uint16_t msg_len;               //!< Message length in bytes(Not including header or CRC)
    uint16_t seq_number;            //!< Counts down from N-1 to 0 for multiple related logs. Reserved to 0x00
    uint8_t idle_time;              //!< Time the processor was idle in last sec between logs with same ID. Reserved to 0x00
    time_status_enum time_status;       //!< Indicates the quality of the GPS time
    uint16_t week;                  //!< GPS Week number
    uint32_t ms_in_week;            //!< Milliseconds into week
    uint32_t receiver_status;       //!< Receiver status word. Reserved to 0x00.
    uint16_t reserved;              //!< Reserved for internal use
    uint16_t receiver_sw_version;   //!< Receiver software build number (0-65535). Reserved to -0xbe0xa2
};

struct __attribute__((__packed__)) tersus_HEADING{
    tersus_Header tersus_header;
    solution_status sol_stat;   //enum
    position_type pos_type; //enum
    float length;           //Baseline Length.
    float heading;          //Heading in degrees. 0 to 360 degrees.
    float pitch;            //Pitch(+-90)
    float reserved;         //Reserved.
    float hdg_std_dev;      //Heading standard deviation in degrees.
    float pitch_std_dev;    //Pitch standard deviationn in degrees.
    int8_t stn_id[4];       //Station ID string.
    uint8_t no_of_sv;       //Number of satellites tracked.
    uint8_t no_of_soln_sv;      //Number of satellites in solution.
    uint8_t no_of_sv_in_obs;    //Number of satellites above the elevation mask angle.
    uint8_t no_of_sv_in_obs_l2; //Number of satellites above the mask angle with L2.
    uint8_t sol_source;         //Solution Source.
    uint8_t ext_sol_stat;       //Extended Solution Status
    uint8_t galileo_bds_sig_mask;   //Galileo & BDS signal mask used
    uint8_t gps_glonass_sig_mask;   //GPS & GLONASS signal mask used.
    uint32_t tersus_crc;                // 32-bit CRC
};

union tersus_message {
    struct tersus_HEADING tersus_heading_t;
    uint8_t raw_data[1024];
};





void processNMEA(uint8_t );
void parseNMEA_msg();
uint8_t nmea_crc_check(uint8_t *, uint8_t );
bool self_isDigit(char );
float self_strtof(char *);
int16_t _from_hex(char );

void processTersus(uint8_t);
void parseTersus(uint16_t);

unsigned long CRC32Value(int );
unsigned long CalculateBlockCRC32( unsigned long, unsigned char *);
unsigned long ByteSwap (unsigned long );


class AP_Tersus {
public:
    //Tersus
    void processGNHDT();

    void init_tersusLogging(void);

    //Tersus heading
    float heading = 0.0;
    uint8_t heading_state = 0;
    float attitude_heading = 0.0;
    uint64_t attitude_headingLog_timeStamp = 0;

    void Log_Write_Tersus();

    void nmea_init();


private:
    //AP_Tersus.cpp


    void tersus_init();
    // void parseTersus(uint16_t );

};

extern AP_Tersus tersus;
