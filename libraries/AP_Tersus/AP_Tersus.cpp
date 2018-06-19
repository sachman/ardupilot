
#include "AP_Tersus.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
/*#include <Copter.h>*/
#include <../ArduCopter/Copter.h>


#define CRC32_POLYNOMIAL 0xEDB88320 // CRC32_POLYNOMIAL.

#define TERSUS_HEADING_DEBUG

enum ascii_state ASCII_state;
enum nmea_state NMEA_state;

uint8_t header_array[HEADER_SIZE], comnav_crc[4];
uint16_t tersus_msg_length, expected_msg_length = 0, tersus_msg_id, raw_data_index, msg_buf_index = 0;

enum tersus_state tersusProcessDataState;

union tersus_message tersus_message_t;

uint8_t ascii_header_index = 0, ASCII_header[20], ASCII_data_buf[512], GPSBuffer[82], GPSIndex = 0, tersus_calc_crc = 0, tersus_rec_crc = 0, count = 0, test_calc_crc = 0, rec_crc_test;
uint8_t crc_count = 0, crc_index = 0;
uint16_t ascii_data_buf_index = 0;

uint32_t calculated_CRC = 0, rec_CRC = 0;



int fd_tersusHeading_logFile  = -1;
bool tersusData_isPresent = false;
uint32_t tersusLog_bytes_written = 0;

int16_t _from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

void Copter::read_tersus_serial()
{
    uint16_t buf_len = 0, i;
    uint8_t serial_nmea[1024];
    buf_len = hal.uartE->available();

    if(buf_len > 0)
    {
        for(i = 0; i < buf_len; i++)
        {
            serial_nmea[i] = hal.uartE->read();
        }

        for(i = 0; i< buf_len; i++)
        {
            // processNMEA(serial_nmea[i]);
            processTersus(serial_nmea[i]);
        }

    }
}

void AP_Tersus::nmea_init()
{
    NMEA_state = START_NMEA;
}

void processNMEA(uint8_t data)
{
    switch(NMEA_state)
    {
        case START_NMEA:
            if ((data =='$') || (GPSIndex >= 255))
            {
                GPSIndex = 0;
                NMEA_state = GET_NMEA_DATA;
                test_calc_crc = 0;
                // ::printf("\nStart NMEA detected\n");
            }
            break;

        case GET_NMEA_DATA:
            if(data != '*')
            {
                GPSBuffer[GPSIndex++] = data;
#ifdef TERSUS_HEADING_DEBUG
                ::printf("%c", GPSBuffer[GPSIndex-1]);
#endif
                test_calc_crc ^= GPSBuffer[GPSIndex-1];
            }else{
                GPSBuffer[GPSIndex] = '\0';
#ifdef TERSUS_HEADING_DEBUG
                ::printf("\nIndex %d\n", GPSIndex);
#endif
                NMEA_state = GET_NMEA_CRC;
            }
            break;

        case GET_NMEA_CRC:
            if(data != '\r')
            {
                count++;
                if(count == 1) rec_crc_test = data;
                if(count == 2)
                {
                    tersus_rec_crc = 16 * _from_hex(rec_crc_test) + _from_hex(data);
#ifdef TERSUS_HEADING_DEBUG
                    ::printf("rec_crc after %X\n", tersus_rec_crc);
                    ::printf("test_calc_crc %X\n", test_calc_crc);
#endif
                    test_calc_crc = 0;
                }
            }else
            {   count = 0;
                NMEA_state = END_DATA;
            }
            break;

        case END_DATA:
            if (data == '\n')
            {
              parseNMEA_msg();
              GPSIndex = 0;
              NMEA_state = START_NMEA;
            }
            break;

         default :
            NMEA_state = START_NMEA;
#ifdef TERSUS_HEADING_DEBUG
            ::printf("default\n");
#endif
    }

}

void parseNMEA_msg()
{
    if ((GPSBuffer[0] == 'G') && (GPSBuffer[1] == 'N' || GPSBuffer[1] == 'P') && (GPSBuffer[2] == 'H') && (GPSBuffer[3] == 'D') && (GPSBuffer[4] == 'T'))
      {
#ifdef TERSUS_HEADING_DEBUG
        ::printf("GNHDT found\n");
#endif
        tersus_calc_crc = nmea_crc_check(GPSBuffer, GPSIndex);
        if(tersus_calc_crc == tersus_rec_crc)
            tersus.processGNHDT();
        else
        {
#ifdef TERSUS_HEADING_DEBUG
            ::printf("Crc mismatch calc_crc %x rec_crc %x\n", tersus_calc_crc, tersus_rec_crc);
#endif
        }
      }
      else
      {
#ifdef TERSUS_HEADING_DEBUG
        ::printf("GNHDT not found\n");
#endif
      }
}

uint8_t nmea_crc_check(uint8_t *test, uint8_t index)
{
     int16_t i;
     uint8_t XOR;
     // const char *Buff = "GPGGA,204502.00,5106.9813,N,11402.2921,W,1,09,0.9,1065.02,M,-16.27,M,,";
     // uint16_t iLen = strlen(Buff);
     uint16_t iLen = index;
     for (XOR = 0, i = 0; i < iLen; i++)
     {
        // ::printf("%x ", XOR);
        XOR ^= (uint8_t)test[i];
     }
#ifdef TERSUS_HEADING_DEBUG
    ::printf("\nCalculated %x\n", XOR);
#endif
    return XOR;
}

bool self_isDigit(char c)
{
    if(c >= '0' && c<= '9')
        return true;
    else return false;
}

void AP_Tersus::processGNHDT()
{
#ifdef TERSUS_HEADING_DEBUG
    ::printf("In process GNHDT\n");
#endif
    char temp_data[12];
    uint8_t i = 0;

    for( i = 0; i < sizeof(temp_data); i++)
        {
            if(GPSBuffer[i+6] == ',') return;

            temp_data[i] = GPSBuffer[i+6];
            if(GPSBuffer[i+7] == ',')
            {
                temp_data[i+1] = '\0';
                break;
            }
        }

        tersus.heading = self_strtof(temp_data);
#ifdef TERSUS_HEADING_DEBUG
        ::printf("Heading %f\n", (double)tersus.heading);
#endif
}

float self_strtof(char *p){

    int32_t num = 0, num_frac = 0;
    uint8_t dec_count = 0;
    float num1 = 0.0f;

    while(*p != '\0'){
        if(self_isDigit(*p)){
            num = (num * 10) + CHAR_TO_DIGIT(*p);
            p++;
        }
        else if( *p == '.'){
                // num1 = num1 + (float)num;
#ifdef TERSUS_HEADING_DEBUG
                ::printf("num1 %f\n", (double)num1);
#endif
//              p++;
                break;
        }
    }

    if(*p == '.'){

        // float frac = 0.1f;
#ifdef TERSUS_HEADING_DEBUG
        ::printf("value *p %c\n", *p);
#endif
        p++;

        while( *p != '\0'){
            ::printf("%d ",CHAR_TO_DIGIT(*p));
            num_frac = (num_frac * 10) + (CHAR_TO_DIGIT(*p));
                dec_count++;
            // frac = frac * 0.1f;
#ifdef TERSUS_HEADING_DEBUG
            ::printf("num_frac %f dec_count %d\n", (double)num_frac, dec_count);
#endif
            p++;
        }

    }

    num1 = float(num) + (float)num_frac/1000;
    ::printf("num %d num_frac %d\n", num, num_frac);

    return num1;
}

void AP_Tersus::tersus_init(){
    tersusProcessDataState = WAIT_TERSUS_SYNC1;
}

unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for (j=8;j>0;j--)
    {
        if (ulCRC & 1)
        ulCRC = (ulCRC >> 1)^CRC32_POLYNOMIAL;
        else ulCRC >>= 1;
    }
    return ulCRC;
}

unsigned long CalculateBlockCRC32(
unsigned long ulCount,
unsigned char *ucBuffer)
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while (ulCount-- != 0)
    {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC^*ucBuffer++)&0xff);
        ulCRC = ulTemp1^ulTemp2;
    }
    return(ulCRC);
}

unsigned long ByteSwap (unsigned long n)
{
    return (((n &0x000000FF)<<24)+(( n &0x0000FF00)<<8)+
    ((n &0x00FF0000)>>8)+(( n &0xFF000000)>>24));
}

void processTersus(uint8_t data){

    switch(tersusProcessDataState){

    case WAIT_TERSUS_SYNC1 :
        if(TERSUS_SYNC_BYTE_1 == data){
            raw_data_index = 0;
            msg_buf_index = 0;
//          chprintf(chp, "i%d D%x, ", raw_data_index, data);
            tersus_message_t.raw_data[raw_data_index++] = data;
//          msg_buf[msg_buf_index++] = data;
            tersusProcessDataState = WAIT_TERSUS_SYNC2;

        }

        break;

    case WAIT_TERSUS_SYNC2 :
        if(TERSUS_SYNC_BYTE_2 == data){
//          chprintf(chp, "i%d D%x, ", raw_data_index, data);
            tersus_message_t.raw_data[raw_data_index++] = data;
//          msg_buf[msg_buf_index++] = data;
            tersusProcessDataState = WAIT_TERSUS_SYNC3;
//          chprintf(chp, "Sync2\n");
        }
        break;

    case WAIT_TERSUS_SYNC3 :
        if(TERSUS_SYNC_BYTE_3 == data){
//          chprintf(chp, "Sync bytes completed\n");
//          chprintf(chp, "i%d D%x Sync done\n", raw_data_index, data);
            tersus_message_t.raw_data[raw_data_index++] = data;
//          msg_buf[msg_buf_index++] = data;
            tersusProcessDataState = GET_TERSUS_HEADER_INFO;
//          chprintf(chp, "Sync3\n");
        }
        break;

    case GET_TERSUS_HEADER_INFO :
//      if(data != ';'){
//          chprintf(chp, "Getting header %c", data);
//          header_array[header_array_index++] = data;

//          if((header_array_index + 1) == HEADER_LEN_INDEX){     // Has header length been stored?
//              if(header_array[HEADER_LEN_INDEX] == HEADER_SIZE) // If received header length is equal to HEADER_SIZE, Cool! //
//                  chprintf(chp, "Header length verified\n");
//              else {                                            // Something went wrong. Start over.
//                  chprintf(chp, "Wrong header length : %d", header_array[HEADER_LEN_INDEX]);
//                  comnavProcessDataState = START_MSG;
//                  processASCIIflag = false;
//              }
//          }
//      }else if(header_array_index == HEADER_SIZE){
//          comnav_msg_length = 0;
//          header_array_index = 0;
//
//          expected_msg_length = header_array[MSG_LENGTH_END_INDEX];
//          expected_msg_length = (expected_msg_length << 8)
//                              + (header_array[MSG_LENGTH_END_INDEX-1]); // Little endian ?
//
//          comnav_msg_id = header_array[MSG_ID_END_INDEX];
//          comnav_msg_id = (comnav_msg_id << 8) + (header_array[MSG_ID_END_INDEX-1]);
//
//          if(expected_msg_length <= sizeof(raw_data))
//              comnavProcessDataState = GET_COMNAV_DATA;
//          else {
//              comnavProcessDataState = START_MSG;
//              processASCIIflag = false;
//          }
//      }

        if(raw_data_index < HEADER_SIZE){
//          chprintf(chp, "i%d D%x, ", raw_data_index, data);
            tersus_message_t.raw_data[raw_data_index++] = data;
//          msg_buf[msg_buf_index++] = data;
        }
//      }else{
//          comnav_msg_length = 0;
//          expected_msg_length = comnav_message_t.comnav_marktime_t.comnav_header.message_length;
//          comnav_msg_id = comnav_message_t.comnav_marktime_t.comnav_header.message_id;

        if(raw_data_index == HEADER_SIZE){
//          chprintf(chp,"\n");
        expected_msg_length = tersus_message_t.raw_data[MSG_LENGTH_END_INDEX];
        expected_msg_length = (expected_msg_length << 8)
                            + (tersus_message_t.raw_data[MSG_LENGTH_END_INDEX-1]); // Little endian ?

        tersus_msg_id = tersus_message_t.raw_data[MSG_ID_END_INDEX];
        tersus_msg_id = (tersus_msg_id << 8) + (tersus_message_t.raw_data[MSG_ID_END_INDEX-1]);
        //Modified. Complete code here
        if(expected_msg_length <= sizeof(tersus_message_t)) tersusProcessDataState = GET_TERSUS_DATA;
        else tersusProcessDataState = WAIT_TERSUS_SYNC1;
//          chprintf(chp, "L%d id%d\n", expected_msg_length, comnav_msg_id);
//          chprintf(chp, "HEADER INFO\n");
        }

        break;

    case GET_TERSUS_DATA :
        if( raw_data_index < (expected_msg_length + HEADER_SIZE) ){
//          chprintf(chp, "i: %d D: %x, ", raw_data_index, data);
            tersus_message_t.raw_data[raw_data_index++] = data;
//          msg_buf[msg_buf_index++] = data;
        }

        if(raw_data_index == (expected_msg_length + HEADER_SIZE)){
//          chprintf(chp,"\n");
//          chprintf(chp, "size %d index %d\n", sizeof(msg_buf), msg_buf_index);
            calculated_CRC = CalculateBlockCRC32(raw_data_index, tersus_message_t.raw_data);
//          cal_CRC_test = CalculateBlockCRC32(msg_buf_index, msg_buf);
            rec_CRC = 0;
//          iLen = msg_buf_index;
//          CRC_test = CalculateBlockCRC32(iLen, msg_buf);
//          chprintf(chp, "CRC %x, byteswap %x\n", CRC_test, ByteSwap(CRC_test));
            tersusProcessDataState = GET_TERSUS_CRC;
//          chprintf(chp, "Data\n");
            crc_count = 4;
        }

        break;

    case GET_TERSUS_CRC :

//      if(data != '\r')
//          comnav_crc[i++] = data;
        if(crc_count){
        crc_count--;
        tersus_message_t.raw_data[raw_data_index++] = data;
//      chprintf(chp, "%x", data);
//      i++;
//      chprintf(chp, "crc_count %d index%d\n", crc_count, raw_data_index);
        }
//      if(data == '\n'){
        if(!crc_count){
//          chprintf(chp, "crc_count %dProcessing done\n", crc_count);
//          i = 0;
            rec_CRC = ((uint32_t)tersus_message_t.raw_data[raw_data_index-4] << 24)
                    + ((uint32_t)tersus_message_t.raw_data[raw_data_index-3] << 16)
                    + ((uint32_t)tersus_message_t.raw_data[raw_data_index-2] << 8)
                    + ((uint32_t)tersus_message_t.raw_data[raw_data_index-1]);
//          chprintf(chp, "\nCalc %x Rec %x\n", calculated_CRC, rec_CRC);
//          chprintf(chp, "Cal CRC test %x\n", cal_CRC_test);
//          chprintf(chp, "Byteswap %x\n", ByteSwap(cal_CRC_test));
            if(ByteSwap(calculated_CRC) == rec_CRC){
                // for(uint16_t i = 0; i < raw_data_index; i++)
                //  ::printf("%X ", tersus_message_t.raw_data[i]);
                // ::printf("\n");
            //Decode the message
            parseTersus(tersus_msg_id);
            tersusProcessDataState = WAIT_TERSUS_SYNC1;
            // processTersusflag = false;
            }
            else {
                ::printf("CRC error. Calc %x vs Rec %x\n", ByteSwap(calculated_CRC), rec_CRC);
                tersusProcessDataState = WAIT_TERSUS_SYNC1;
            }
        }
//      chprintf(chp, "i%d D%x, ", raw_data_index, data);
//      chprintf(chp, "exit CRC %x \n", data);
//      chprintf(chp,"\n");
//      }
        break;

    default :
        tersusProcessDataState = WAIT_TERSUS_SYNC1;
//      chprintf(chp, "Default %d\n", comnavProcessDataState);
        break;
    }
//  chprintf(chp, "State %d\n", comnavProcessDataState);

//  }//End for

//  chprintf(chp, "i %d numc %d", i , numc);
}

void parseTersus(uint16_t parse_tersus_msg_id){

    switch(parse_tersus_msg_id){

    case TERSUS_HEADING_MSG_ID :
        // sysTime_ms = ST2MS((uint64_t)chVTGetSystemTime());
        // seconds_test = comnav_message_t.comnav_marktime_t.seconds;
        // week_test = comnav_message_t.comnav_marktime_t.week;
        // cam_log_flag = true;
//      chprintf(chp, "Marktime msg!Seconds %f week %d\n", (float)seconds_test, week_test);
        tersus.heading = tersus_message_t.tersus_heading_t.heading;


        /* Constant offset correction for Dual antenna heading to align with UAV's Heading Axis.
         * The offset value corrected here is approximate.
         */
        tersus.heading -= 90;
        if (tersus.heading < 0)
            tersus.heading = 360 + tersus.heading;

        tersus.sol_state = tersus_message_t.tersus_heading_t.sol_stat;
        tersus.base_length = tersus_message_t.tersus_heading_t.length;
        tersus.pitch = tersus_message_t.tersus_heading_t.pitch;
        tersus.hdg_std_dev = tersus_message_t.tersus_heading_t.hdg_std_dev;
        tersus.satCount_track = tersus_message_t.tersus_heading_t.no_of_sv;
        tersus.satCount_sol = tersus_message_t.tersus_heading_t.no_of_soln_sv;
        tersus.heading_state = tersus_message_t.tersus_heading_t.pos_type;

        tersusData_isPresent = true;
#ifdef TERSUS_HEADING_DEBUG
        ::printf("Heading received %f\n", (double)tersus.heading);
        ::printf("T %d ID %d\n",tersus_message_t.tersus_heading_t.tersus_header.ms_in_week, tersus_message_t.tersus_heading_t.tersus_header.msg_id);
        ::printf("L %d pitch %f\n", tersus_message_t.tersus_heading_t.tersus_header.msg_len, tersus_message_t.tersus_heading_t.pitch);
        ::printf("SV %d\n", tersus_message_t.tersus_heading_t.no_of_sv);
        ::printf("SV_in_Sol %d\n", tersus.satCount_sol);
        ::printf("Case Length: %3.2f\n", tersus.base_length);
        ::printf("Heading SD: %3.2f\n", tersus.hdg_std_dev);
#endif


#ifdef TERSUS_HEADING_DEBUG
        ::printf("Sol type %d Pos type %d\n", tersus.sol_state, tersus.heading_state);
#endif
        /*
        if(tersus_message_t.tersus_heading_t.pos_type == TERSUS_POS_FIXED){
            tersus.heading_state = 1;
#ifdef TERSUS_HEADING_DEBUG
            ::printf("Fixed %d Val %d\n", tersus.heading_state, tersus_message_t.tersus_heading_t.pos_type);
#endif
        }
        else if(tersus_message_t.tersus_heading_t.pos_type == TERSUS_POS_FLOAT){
            tersus.heading_state = 0;
#ifdef TERSUS_HEADING_DEBUG
            ::printf("Float %d Val %d\n", tersus.heading_state, tersus_message_t.tersus_heading_t.pos_type);
#endif
        }
        else {
            tersus.heading_state = 0;
#ifdef TERSUS_HEADING_DEBUG
            ::printf("Unknown %d Val %d\n", tersus.heading_state, tersus_message_t.tersus_heading_t.pos_type);
#endif
        }
        */
        break;

    default :
#ifdef TERSUS_HEADING_DEBUG
        ::printf("Default %d\n", tersus_msg_id);
#endif
        break;

    }
}

/*void processASCII(uint8_t data){

    //Initialization of ASCII_state is pending.
    switch(ASCII_state){

    case START_ASCII :
        if(data == '$'){
            ascii_header_index = 0;
            ASCII_state = GET_ASCII_HEADER;
        }
        break;

    case GET_ASCII_HEADER :
        if(data != ','){
            ASCII_header[ascii_header_index++] = data;
        }else if(data == ','){
            ascii_data_buf_index  = 0;
            ASCII_state = GET_ASCII_DATA;
        }
        break;

    case GET_ASCII_DATA :
        if(data != '*'){
            ASCII_data_buf[ascii_data_buf_index++] = data;
        }else if(data == '*'){
            ASCII_state = GET_ASCII_CRC;
        }
        break;

    case GET_ASCII_CRC :
        if(data != '\r'){
            comnav_crc[crc_index++] = data;
        }
        else if(data == '\r'){
            ASCII_state = END_ASCII;
        }
        break;

    case END_ASCII :
        if(data == '\n'){
            parseASCIIdata();
            ASCII_state = START_ASCII;
        }
        break;

    default :
        ASCII_state = START_ASCII;
        break;

    }
}*/

void Copter::log_tersusHeading(void)
{
    uint16_t bytes_written = 0;
    char heading_data[100];
    if (tersusData_isPresent == true)
    {
        ::printf("Tersus Heading Data received\n");
        if (fd_tersusHeading_logFile < 0)
            fd_tersusHeading_logFile = open(TERSUS_HEADING_LOG_FILE, O_RDWR, 0644);
        if (fd_tersusHeading_logFile > 0)
        {
            sprintf(heading_data, "%llu, %d, %3.2f ,%llu ,%3.2f\n",
                    AP_HAL::micros64(), tersus.heading_state, tersus.heading,
                    tersus.attitude_headingLog_timeStamp, float(tersus.attitude_heading)/100);
            lseek(fd_tersusHeading_logFile, tersusLog_bytes_written, SEEK_SET);
            bytes_written = write(fd_tersusHeading_logFile, heading_data, strlen(heading_data));
            tersusLog_bytes_written += bytes_written;
            if (bytes_written <= 0)
#ifdef TERSUS_HEADING_DEBUG
                ::printf("Heading Data not logged.\n");
#endif
            close(fd_tersusHeading_logFile);
            fd_tersusHeading_logFile = -1;
        }
        else
#ifdef TERSUS_HEADING_DEBUG
            ::printf("Error opening File %s for Tersus Logging\n", TERSUS_HEADING_LOG_FILE);
#endif

        tersusData_isPresent = false;
    }

}

void AP_Tersus::init_tersusLogging(void)
{
    /*hal.console->printf("Into %s\n", __func__);*/
    fd_tersusHeading_logFile = open(TERSUS_HEADING_LOG_FILE, O_RDWR|O_CREAT|O_TRUNC, 0644);
    if (fd_tersusHeading_logFile < 0)
    {
#ifdef TERSUS_HEADING_DEBUG
       ::printf("Error in opening/creating file %s for Tersus Heading data Logging\n", TERSUS_HEADING_LOG_FILE);
#endif
    }
}

AP_Tersus tersus;

