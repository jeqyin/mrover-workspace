#include "imu.hpp"

Imu::Imu(PinName sda, PinName scl) :
    i2c_(sda, scl),
    valid_(false)
{
    memset(&this->accel_, 0, sizeof(this->accel_));
    memset(&this->mag_, 0, sizeof(this->mag_));
    // Ensure that it's properly connected
    uint8_t id = this->read_reg(REGISTER_WHO_AM_I);
    if (id != ID) {
        return; // valid_ = false
    }

    // Set the IMU to standby
    this->write_reg(REGISTER_CTRL_REG1, 0);

    // TODO make the range customizable (rn hard-coded for 4G)
    this->write_reg(REGISTER_XYZ_DATA_CFG, 0x01);

    // Enable high-resolution mode
    this->write_reg(REGISTER_CTRL_REG2, 0x02);
    // Active, Normal, Low-noise, 100 Hz, hybrid
    this->write_reg(REGISTER_CTRL_REG1, 0x15);

    // Configure magnetometer
    // Hybrid, oversampling rate=16
    this->write_reg(REGISTER_MCTRL_REG1, 0x1F);
    // Jump to reg 0x33 after reading 0x06
    this->write_reg(REGISTER_MCTRL_REG2, 0x20);

    this->valid_ = true;
}

bool Imu::valid() const {
    return this->valid_;
}

void Imu::read() {
    memset(&this->accel_, 0, sizeof(this->accel_));
    memset(&this->mag_, 0, sizeof(this->mag_));

    char msg = REGISTER_STATUS | 0x80;
    this->i2c_.write(ADDRESS, &msg, 1);

    /*
     *  [0] = status
     *  [1] = Accelerometer X high byte
     *  [2] = Accelerometer X low byte
     *  [3] = Accelerometer Y high byte
     *  [4] = Accelerometer Y low byte
     *  [5] = Accelerometer Z high byte
     *  [6] = Accelerometer Z low byte
     *  [7] = Magnetometer X high byte
     *  [8] = Magnetometer X low byte
     *  [9] = Magnetometer Y high byte
     * [10] = Magnetometer Y low byte
     * [11] = Magnetometer Z high byte
     * [12] = Magnetometer Z low byte
     */
    char recvBuf[13];
    this->i2c_.read(ADDRESS, recvBuf, 13);

    int16_t accel_x = (int16_t)((recvBuf[1] << 8) | recvBuf[2]) >> 2;
    int16_t accel_y = (int16_t)((recvBuf[3] << 8) | recvBuf[4]) >> 2;
    int16_t accel_z = (int16_t)((recvBuf[5] << 8) | recvBuf[6]) >> 2;

    int16_t mag_x = (int16_t)((recvBuf[7] << 8) | recvBuf[8]);
    int16_t mag_y = (int16_t)((recvBuf[9] << 8) | recvBuf[10]);
    int16_t mag_z = (int16_t)((recvBuf[11] << 8) | recvBuf[12]);

    this->accel_.x = accel_x * ACCEL_MG_LSB_4G * GRAVITY_STANDARD;
    this->accel_.y = accel_y * ACCEL_MG_LSB_4G * GRAVITY_STANDARD;
    this->accel_.z = accel_z * ACCEL_MG_LSB_4G * GRAVITY_STANDARD;

    this->mag_.x = mag_x * MAG_UT_LSB;
    this->mag_.y = mag_y * MAG_UT_LSB;
    this->mag_.z = mag_z * MAG_UT_LSB;
}

Vector3f Imu::accelerometer() const {
    return this->accel_;
}

Vector3f Imu::magnetometer() const {
    return this->mag_;
}

void Imu::write_reg(uint8_t reg, uint8_t value) {
    char buf[2] = {reg, value};
    this->i2c_.write(ADDRESS, buf, 2);
}

uint8_t Imu::read_reg(uint8_t reg) {
    char signed_reg = (char) reg;
    this->i2c_.write(ADDRESS, &signed_reg, 1);
    return (uint8_t)this->i2c_.read(ADDRESS);
}
