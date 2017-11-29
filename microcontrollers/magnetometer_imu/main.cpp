#include "mbed.h"
#include "imu.hpp"

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);
Serial dbg(USBTX, USBRX, 9600);
Imu imu(PB_7, PB_6);

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
        Vector3f a = imu.accelerometer();
        Vector3f m = imu.magnetometer();
        Vector3f g = imu.gyroscope();
        dbg.printf("A: (x=%.4f, y=%.4f, z=%.4f) m/s^2\r\n", a.x, a.y, a.z);
        dbg.printf("M: (x=%.4f, y=%.4f, z=%.4f) uT\r\n", m.x, m.y, m.z);
        dbg.printf("G: (x=%.4f, y=%.4f, z=%.4f) deg/s\r\n", g.x, g.y, g.z);

        wait(0.5);
    }
}
