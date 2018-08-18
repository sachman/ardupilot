/*
 * AP_InertialSensor_HG1120.h
 *
 *  Created on: 04-Aug-2018
 *      Author: sachin
 */

#ifndef LIBRARIES_AP_INERTIALSENSOR_AP_INERTIALSENSOR_HG1120_H_
#define LIBRARIES_AP_INERTIALSENSOR_AP_INERTIALSENSOR_HG1120_H_


#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/* enable debug to see a register dump on startup */
#define HG1120_DEBUG 0

class AP_InertialSensor_HG1120 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_HG1120() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);
private:
    AP_InertialSensor_HG1120(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                              int drdy_pin_num_xg,
                              enum Rotation rotation);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_8G,
        A_SCALE_16G
    };

//    void _poll_data();
//    void _fifo_reset();

    bool _init_sensor();
    bool _hardware_init();

//    void _gyro_init();
//    void _accel_init();
//
//    void _set_gyro_scale();
//    void _set_accel_scale(accel_scale scale);

//    uint8_t _register_read(uint8_t reg);
    uint8_t _getIMUaddress();
//    void _register_write(uint8_t reg, uint8_t val, bool checked=false);
//
//    void _read_data_transaction_x(uint16_t samples);
//    void _read_data_transaction_g(uint16_t samples);

    void honWl_IMU_spiFetch(uint8_t *data_out);

    bool parse_HonWlIMU_data(uint8_t *data_in);

//    bool _poll_data();
    void _poll_data();

    #if HG1120_DEBUG
    void        _dump_registers();
    #endif

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_spi_sem;
    AP_HAL::DigitalSource * _drdy_pin_xg;
    float _gyro_scale;
    float _accel_scale;
    int _drdy_pin_num_xg;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
    enum Rotation _rotation;

//    union HG1120_message imuMessage;

    uint8_t _imu_address;
    double _angularRate_X, _angularRate_Y, _angularRate_Z;
    double _accel_X, _accel_Y, _accel_Z;
    double _mag_X, _mag_Y, _mag_Z;




    struct control_data {
        uint8_t identifiers[2];
        uint8_t angular_rate_X[2];
        uint8_t angular_rate_Y[2];
        uint8_t angular_rate_Z[2];
        uint8_t lin_accel_X[2];
        uint8_t lin_accel_Y[2];
        uint8_t lin_accel_Z[2];
        uint8_t mag_X[2];
        uint8_t mag_Y[2];
        uint8_t mag_Z[2];
        uint8_t main_status_word[2];
        uint8_t mux_status_word[2];
    };

    union control_data_union {
        uint8_t data[12][2];
        struct control_data controlData;
    };

    struct inertial_data {
        uint8_t delta_angle_X[4];
        uint8_t delta_angle_Y[4];
        uint8_t delta_angle_Z[4];
        uint8_t delta_velocity_X[4];
        uint8_t delta_velocity_Y[4];
        uint8_t delta_velocity_Z[4];
    };

    struct identity {
        uint8_t imu_address;
        uint8_t message_id;
    };

    struct control_and_nav_data_05format {
        union control_data_union controlData;
        struct inertial_data inertialData;
        uint8_t checkSum[2];
    };

    struct spi_InertialMessage_05format {
        uint8_t spi_data_size;
        struct control_and_nav_data_05format controlAndNavData;
    };

    struct control_message_04format {
        union control_data_union controlData;
        uint8_t checkSum[2];
    };

    struct spi_spare_data_04format {
        uint8_t spare_bytes[24];
    };

    struct spi_controlMessage_04format {
        uint8_t spi_data_size;
        struct control_message_04format controlMessage;
        struct spi_spare_data_04format spareData;
    };

    union HG1120_message {
//        uint8_t raw_data[100];
//        uint8_t raw_data[SENSOR_MSG_BUFFER_LENGTH];
        uint8_t raw_data[700];
        struct spi_controlMessage_04format spiCtrlMsgPacket;
        struct spi_InertialMessage_05format spiInertialMsgPacket;
    };

    HG1120_message imuMessage;
};



#endif /* LIBRARIES_AP_INERTIALSENSOR_AP_INERTIALSENSOR_HG1120_H_ */
