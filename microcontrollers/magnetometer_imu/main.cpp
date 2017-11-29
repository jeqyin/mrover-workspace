#include "mbed.h"
#include "imu.hpp"

Serial dbg(USBTX, USBRX, 9600);
Imu imu(PB_7, PB_6);

int main() {
    while (true) {
        imu.read();
        Vector3f a = imu.accelerometer();
        Vector3f m = imu.magnetometer();
        dbg.printf("A: (x=%.4f, y=%.4f, z=%.4f) m/s^2\n", a.x, a.y, a.z);
        dbg.printf("M: (x=%.4f, y=%.4f, z=%.4f) uT\n", m.x, m.y, m.z);

        wait(0.5);
    }
}
