/*
 * AP_InertialSensor_HG1120.cpp
 *
 *  Created on: 04-Aug-2018
 *      Author: sachin
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_HG1120.h"

#include <utility>

#include <AP_HAL/GPIO.h>

extern const AP_HAL::HAL& hal;

#define WHO_AM_I                            0x0E
#define DATA_RDY_LEVEL                      1
#define CONTROL_MSG_ID                      0x04
#define HG1120_DRY_XG_PIN                   -1
#define SENSOR_MESSAGE_PACKET_LENGTH        51
#define CONTROL_MSG_SPI_PACKET_LENGTH_FIELD 0x1A

#define SENSOR_MESSAGE_BUFFER_LENGTH    700

uint8_t sensor_spi_raw_data[SENSOR_MESSAGE_BUFFER_LENGTH];


AP_InertialSensor_HG1120::AP_InertialSensor_HG1120(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                     int drdy_pin_num_xg,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _drdy_pin_num_xg(drdy_pin_num_xg)
    , _rotation(rotation)

{
}

AP_InertialSensor_Backend *AP_InertialSensor_HG1120::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_HG1120 *sensor =
        new AP_InertialSensor_HG1120(_imu,std::move(dev),
                                      HG1120_DRY_XG_PIN,
                                      rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_HG1120::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();

    if (_drdy_pin_num_xg >= 0) {
        _drdy_pin_xg = hal.gpio->channel(_drdy_pin_num_xg);
        if (_drdy_pin_xg == nullptr) {
            AP_HAL::panic("HG1120: null accel data-ready GPIO channel\n");
        }
        _drdy_pin_xg->mode(HAL_GPIO_INPUT);
    }

    bool success = _hardware_init();

#if HG1120_DEBUG
    _dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_HG1120::_hardware_init()
{
    if (!_spi_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    uint8_t tries, whoami;

    for (tries = 0; tries < 20; tries++) {

        whoami = _getIMUaddress();
        if (whoami != WHO_AM_I) {
            hal.console->printf("HG1120: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        }
        if ((whoami == WHO_AM_I) &&
           (imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.identifiers[1]
            == CONTROL_MSG_ID)) {
            break;
        }
        hal.scheduler->delay(1);

#if HG1120_DEBUG
        _dump_registers();
#endif
    }

    if (tries == 20) {
        hal.console->printf("Failed to boot HG1120 20 times\n\n");
        goto fail_tries;
    }
    _spi_sem->give();
    return true;

fail_tries:
    _spi_sem->give();
    return false;
}

uint8_t AP_InertialSensor_HG1120::_getIMUaddress() {
    bool dataPacket_isValid = false;

    if (_drdy_pin_num_xg >= 0) {
        /* If the sensor says data is ready  */
        if (DATA_RDY_LEVEL == hal.gpio->read(_drdy_pin_num_xg)) {
            /* Fetch a data message from the sensor  */
            honWl_IMU_spiFetch(sensor_spi_raw_data);
        }
    }
    else {
        /* Fetch a data message from the sensor  */
        honWl_IMU_spiFetch(sensor_spi_raw_data);
    }

    /* Parse the data */
    dataPacket_isValid = parse_HonWlIMU_data(sensor_spi_raw_data);
   /* If the data packet received was corrupted  */
   if (!dataPacket_isValid) {
       _imu_address = 0;
       return _imu_address;
   }
   _imu_address =
   imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.identifiers[0];

   return _imu_address;
}


bool AP_InertialSensor_HG1120::parse_HonWlIMU_data(uint8_t *data_in) {
    uint8_t j;
    uint16_t u16sum = 0;

    int16_t helper_angularRate_X, helper_angularRate_Y, helper_angularRate_Z;
    int16_t helper_accel_X, helper_accel_Y, helper_accel_Z;
    int16_t helper_mag_X, helper_mag_Y, helper_mag_Z;

    int i = 0;


    /* Extract a Control Msg Packet from the Buffer which contains SPI data
     * as received from sensor.
     */
    for (i=2 ; i<SENSOR_MESSAGE_BUFFER_LENGTH; i++) {
        if ((CONTROL_MSG_ID == data_in[i])
                &&
            (WHO_AM_I == data_in[i-1])
                &&
            (CONTROL_MSG_SPI_PACKET_LENGTH_FIELD == data_in[i-2])
                &&
            /* Ensure that the whole control packet lies within the spi raw data buffer.
             *
             */
            ((i-2 + CONTROL_MSG_SPI_PACKET_LENGTH_FIELD+24)) <
            SENSOR_MESSAGE_BUFFER_LENGTH) {
            /* Copy the control data packet for processing  */
            memcpy(imuMessage.raw_data, &data_in[i-2],
                    CONTROL_MSG_SPI_PACKET_LENGTH_FIELD+24+1);
#if 1
            /* Print the message extracted from the buffer  */
            hal.console->printf("Data Extracted = ");
            for (int k=0; k< SENSOR_MESSAGE_PACKET_LENGTH; k++) {
                hal.console->printf("%02x ", imuMessage.raw_data[k]);
            }
            hal.console->printf("\n");
#endif
            break;
        }
    }

    if (i == SENSOR_MESSAGE_BUFFER_LENGTH) {
        hal.console->printf("HG1120: Latest control data packet not fetched\n\n");
        _imu_address = 0;
        return false;
    }


    const uint16_t receivedCheckSum =
            ((imuMessage.spiCtrlMsgPacket.controlMessage.checkSum[1] << 8 ) |
              imuMessage.spiCtrlMsgPacket.controlMessage.checkSum[0]);


//    /* If the message is of type Control */
//   if (data_in[2] == CONTROL_MSG_ID) {
       /* Calculate checksum from the message bytes received  */
       for (j=0; j< 12; j++) {
           u16sum += ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.data[j][1] << 8) |
                   imuMessage.spiCtrlMsgPacket.controlMessage.controlData.data[j][0]);
       }
       /* Sanity check  */
       if (u16sum != receivedCheckSum) {
           _imu_address = 0;
           hal.console->printf("HG1120: Checksum Error\n\n");
           return false;
       }
       else { /* Data is sane  */
           helper_angularRate_X = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.angular_rate_X[1] << 8) |
                             imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.angular_rate_X[0]);
           helper_angularRate_Y = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.angular_rate_Y[1] << 8) |
                             imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.angular_rate_Y[0]);
           helper_angularRate_Z = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.angular_rate_Z[1] << 8) |
                             imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.angular_rate_Z[0]);
           helper_accel_X = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.lin_accel_X[1] << 8) |
                       imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.lin_accel_X[0]);
           helper_accel_Y = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.lin_accel_Y[1] << 8) |
                       imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.lin_accel_Y[0]);
           helper_accel_Z = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.lin_accel_Z[1] << 8) |
                       imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.lin_accel_Z[0]);
           helper_mag_X = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.mag_X[1] << 8) |
                     imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.mag_X[0]);
           helper_mag_Y = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.mag_Y[1] << 8) |
                     imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.mag_Y[0]);
           helper_mag_Z = ((imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.mag_Z[1] << 8) |
                     imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.mag_Z[0]);

           /* Applying the units and weights as given in the sensor data-sheet  */
           /* Angular rates in rad/s  */
           _angularRate_X = helper_angularRate_X * ( ((double)(pow(2,-20) * 1800 * 2)/3.0) );
           _angularRate_Y = helper_angularRate_Y * ( ((double)(pow(2,-20) * 1800 * 2)/3.0) );
           _angularRate_Z = helper_angularRate_Z * ( ((double)(pow(2,-20) * 1800 * 2)/3.0) );
           /* Accelerations in m/s^2 (meters per second square)  */
           _accel_X = helper_accel_X * ( ((double)(pow(2,-14) * 1800 * 2)/3.0) * 0.3048);
           _accel_Y = helper_accel_Y * ( ((double)(pow(2,-14) * 1800 * 2)/3.0) * 0.3048);
           _accel_Z = helper_accel_Z * ( ((double)(pow(2,-14) * 1800 * 2)/3.0) * 0.3048);
           /* Magnetic fields in Milli-gauss  */
           _mag_X = helper_mag_X * (0.438404);
           _mag_Y = helper_mag_Y * (0.438404);
           _mag_Z = helper_mag_Z * (0.438404);
       }
//   }

   return true;
}

/*
  start the sensor going
 */
void AP_InertialSensor_HG1120::start(void)
{
    _gyro_instance = _imu.register_gyro(952, _dev->get_bus_id_devtype(DEVTYPE_GYR_HG1120));
    _accel_instance = _imu.register_accel(952, _dev->get_bus_id_devtype(DEVTYPE_ACC_HG1120));

    set_accel_orientation(_accel_instance, _rotation);
    set_gyro_orientation(_gyro_instance, _rotation);

    _set_accel_max_abs_offset(_accel_instance, 5.0f);

    /* start the timer process to read samples */
    _dev->register_periodic_callback(1000000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_HG1120::_poll_data, void));
}

/**
 * Timer process to poll for new data from the HG1120.
 */
void AP_InertialSensor_HG1120::_poll_data() {
    hal.console->printf("Into %s\n\n", __func__);
    bool dataPacket_isValid = false;
    int tries = 0;

    for (tries=0; tries<20; tries++) {
        if (_drdy_pin_num_xg >= 0) {
            /* If the sensor says data is ready  */
            if (DATA_RDY_LEVEL == hal.gpio->read(_drdy_pin_num_xg)) {
                /* Fetch a data message from the sensor  */
                honWl_IMU_spiFetch(sensor_spi_raw_data);
            }
        }
        else {
            /* Fetch a data message from the sensor  */
            honWl_IMU_spiFetch(sensor_spi_raw_data);
        }
    }

    /* Check if a control data packet extraction was successful.
     */
    if (tries == 20) {
        hal.console->printf("HG1120: Control data packet could not be fetched\n\n");
        _imu_address = 0;
        return;
    }

    /* Parse the data */
    dataPacket_isValid = parse_HonWlIMU_data(sensor_spi_raw_data);
   /* If the data packet received was corrupted  */
   if (!dataPacket_isValid) {
       _imu_address = 0;
       return;
   }
   _imu_address =
   imuMessage.spiCtrlMsgPacket.controlMessage.controlData.controlData.identifiers[0];

   Vector3f gyro_data(_angularRate_X, _angularRate_Y, //-raw_data.z);
           _angularRate_Z);
   _rotate_and_correct_gyro(_gyro_instance, gyro_data);
   _notify_new_gyro_raw_sample(_gyro_instance, gyro_data);

   Vector3f accel_data(_accel_X, _accel_Y, //-raw_data.z);
           _accel_Z);
   _rotate_and_correct_accel(_accel_instance, accel_data);
   _notify_new_accel_raw_sample(_accel_instance, accel_data);

//   return true;
   return;

}

void AP_InertialSensor_HG1120::honWl_IMU_spiFetch(uint8_t *data_out) {

    const uint8_t txbuf_dummy[SENSOR_MESSAGE_BUFFER_LENGTH] = {0};
//    if (!_dev->transfer(txbuf_dummy, SENSOR_MESSAGE_PACKET_LENGTH,
//            data_out, SENSOR_MESSAGE_PACKET_LENGTH)) {
    if (!_dev->transfer(txbuf_dummy, SENSOR_MESSAGE_BUFFER_LENGTH,
                data_out, SENSOR_MESSAGE_BUFFER_LENGTH)) {
        hal.console->printf("HG1120: error reading from sensor\n");
        return;
    }
}

bool AP_InertialSensor_HG1120::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

#if HG1120_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_LSM9DS1::_dump_registers(void)
{
    hal.console->println("LSM9DS1 registers:");

    const uint8_t first = LSM9DS1XG_ACT_THS;
    const uint8_t last = LSM9DS1XG_INT_GEN_DUR_G;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif


