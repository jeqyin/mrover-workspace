#pragma once

namespace Math {
    struct Quaternion {
        float q0;
        float q1;
        float q2;
        float q3;
    };

    struct Vector3f {
        float x;
        float y;
        float z;
    };

    struct Rot3f {
        float roll;
        float pitch;
        float yaw;
    };

    Rot3f quat_to_rpy(Quaternion q);
}
