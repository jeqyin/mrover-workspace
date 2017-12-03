#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"
#include "madgwick.hpp"

#define DT 0.5

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);
Serial dbg(USBTX, USBRX, 115200);
Imu imu(PB_7, PB_6);
//Madgwick filter(0.25f, 0.0f);

inline float SIGN(float x) {
    if (x >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

Math::Quaternion compute_orientation(
        Math::Vector3f a, Math::Vector3f m) {
    Math::Vector3f H = Math::cross(m, a);

    Math::normalize_vec(H);
    Math::normalize_vec(a);

    Math::Vector3f M = Math::cross(a, H);

    Math::Quaternion q;
    q.q0 = ( H.x + M.y + a.z + 1.0f) / 4.0f;
    q.q1 = ( H.x - M.y - a.z + 1.0f) / 4.0f;
    q.q2 = (-H.x + M.y - a.z + 1.0f) / 4.0f;
    q.q3 = (-H.x - M.y + a.z + 1.0f) / 4.0f;

    if (q.q0 < 0.0f) q.q0 = 0.0f;
    if (q.q1 < 0.0f) q.q1 = 0.0f;
    if (q.q2 < 0.0f) q.q2 = 0.0f;
    if (q.q3 < 0.0f) q.q3 = 0.0f;

    q.q0 = sqrtf(q.q0);
    q.q1 = sqrtf(q.q1);
    q.q2 = sqrtf(q.q2);
    q.q3 = sqrtf(q.q3);

    if (q.q0 >= q.q1 && q.q0 >= q.q2 && q.q0 >= q.q3) {
        q.q1 *= SIGN(M.z - a.y);
        q.q2 *= SIGN(a.x - H.z);
        q.q3 *= SIGN(H.y - M.x);
    } else if (q.q1 >= q.q0 && q.q1 >= q.q2 && q.q1 >= q.q3) {
        q.q0 *= SIGN(M.z - a.y);
        q.q2 *= SIGN(H.y + M.x);
        q.q3 *= SIGN(a.x + H.z);
    } else if (q.q2 >= q.q0 && q.q2 >= q.q1 && q.q2 >= q.q3) {
        q.q0 *= SIGN(a.x - H.z);
        q.q1 *= SIGN(H.y + M.x);
        q.q3 *= SIGN(M.z + a.y);
    } else if (q.q3 >= q.q0 && q.q3 >= q.q1 && q.q3 >= q.q2) {
        q.q0 *= SIGN(H.y - M.x);
        q.q1 *= SIGN(H.z + a.x);
        q.q2 *= SIGN(M.z + a.y);
    } else {
        dbg.printf("ERROR! not all cases covered!\r\n");
    }
    Math::normalize_quat(q);
    return q;
}

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

        //Math::Quaternion quat = filter.update(g, a, m, DT);
        Math::Quaternion quat = compute_orientation(a, m);
        dbg.printf("Orientation: (%.4f, %.4f, %.4f, %.4f)\r\n",
                   quat.q0, quat.q1, quat.q2, quat.q3);

        // convert to cardinal direction
        Math::Rot3f rpy = Math::quat_to_rpy(quat);
        float x_h = m.x*cosf(rpy.pitch) + m.y*sinf(rpy.roll)*sinf(rpy.pitch) - m.z*cosf(rpy.roll)*sinf(rpy.pitch);
        float y_h = m.y*cosf(rpy.roll) - m.z*sinf(rpy.roll);
        float bearing = atan2f(y_h, x_h) * (180.0f/M_PI);
        if (bearing < 0) {
            bearing += 360.0f;
        }
        dbg.printf("Bearing: %.4f\r\n", bearing);

        wait(DT);
    }
}
