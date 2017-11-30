#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"
#include "madgwick.hpp"

#define DT 0.5

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);
Serial dbg(USBTX, USBRX, 9600);
Imu imu(PB_7, PB_6);
Madgwick filter(0.1f, 1.0f/512.0f);

int main() {
    while (true) {
        if (!imu.valid()) {
            green_led = false;
            red_led = true;
            dbg.printf("invalid IMU, check connection\r\n");
            imu.init();
            dbg.printf("reconnecting...\r\n");
            wait(1.0);
            continue;
        } else {
            red_led = false;
            green_led = true;
        }
        imu.read();
        Math::Vector3f a = imu.accelerometer();
        Math::Vector3f m = imu.magnetometer();
        Math::Vector3f g = imu.gyroscope();
        dbg.printf("-----------RAW VALUES------------\r\n");
        dbg.printf("A: (x=%.4f, y=%.4f, z=%.4f) m/s^2\r\n", a.x, a.y, a.z);
        dbg.printf("M: (x=%.4f, y=%.4f, z=%.4f) uT\r\n", m.x, m.y, m.z);
        dbg.printf("G: (x=%.4f, y=%.4f, z=%.4f) deg/s\r\n", g.x, g.y, g.z);
        dbg.printf("\r\n");

        Math::Quaternion quat = filter.update(g, a, m, DT);
        Math::Rot3f rot = Math::quat_to_rpy(quat);
        dbg.printf("Orientation: (roll=%.2f, pitch=%.2f, yaw=%.2f)\r\n", rot.roll, rot.pitch, rot.yaw);

        wait(DT);
    }
}
