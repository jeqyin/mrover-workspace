#pragma once

#include "mbed.h"

struct Vector3f {
    float x;
    float y;
    float z;
};

class Imu {
    public:
        Imu(PinName sda, PinName scl);

        bool valid() const;

        void read();

        Vector3f accelerometer() const;
        Vector3f magnetometer() const;

    private:
        // Address and self-reported ID
        static const uint8_t ADDRESS = 0x1F;
        static const uint8_t ID = 0xC7;

        // Registers
        static const uint8_t REGISTER_STATUS = 0x00;
        static const uint8_t REGISTER_OUT_X_MSB = 0x01;
        static const uint8_t REGISTER_OUT_X_LSB = 0x02;
        static const uint8_t REGISTER_OUT_Y_MSB = 0x03;
        static const uint8_t REGISTER_OUT_Y_LSB = 0x04;
        static const uint8_t REGISTER_OUT_Z_MSB = 0x05;
        static const uint8_t REGISTER_OUT_Z_LSB = 0x06;
        static const uint8_t REGISTER_WHO_AM_I = 0x0D;
        static const uint8_t REGISTER_XYZ_DATA_CFG = 0x0E;
        static const uint8_t REGISTER_CTRL_REG1 = 0x2A;
        static const uint8_t REGISTER_CTRL_REG2 = 0x2B;
        static const uint8_t REGISTER_CTRL_REG3 = 0x2C;
        static const uint8_t REGISTER_CTRL_REG4 = 0x2D;
        static const uint8_t REGISTER_CTRL_REG5 = 0x2E;
        static const uint8_t REGISTER_MSTATUS = 0x32;
        static const uint8_t REGISTER_MOUT_X_MSB = 0x33;
        static const uint8_t REGISTER_MOUT_X_LSB = 0x34;
        static const uint8_t REGISTER_MOUT_Y_MSB = 0x35;
        static const uint8_t REGISTER_MOUT_Y_LSB = 0x36;
        static const uint8_t REGISTER_MOUT_Z_MSB = 0x37;
        static const uint8_t REGISTER_MOUT_Z_LSB = 0x38;
        static const uint8_t REGISTER_MCTRL_REG1 = 0x5B;
        static const uint8_t REGISTER_MCTRL_REG2 = 0x5C;
        static const uint8_t REGISTER_MCTRL_REG3 = 0x5D;
        
        // Unit conversions
        static const float ACCEL_MG_LSB_4G = 0.000488f;
        static const float MAG_UT_LSB = 0.1f;
        static const float GRAVITY_STANDARD = 9.80665f;

        void write_reg(uint8_t reg, uint8_t value);
        uint8_t read_reg(uint8_t reg);

        I2C i2c_;

        Vector3f accel_;
        Vector3f mag_;

        bool valid_;
};
