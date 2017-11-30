#include <math.h>
#include "math_3d.hpp"

Math::Rot3f Math::quat_to_rpy(Math::Quaternion q) {
    Math::Rot3f rot;
    rot.roll = atan2f(q.q0*q.q1 + q.q2*q.q3, 0.5f - q.q1*q.q1 - q.q2*q.q2);
    rot.pitch = asinf(-2.0f * (q.q1*q.q3 - q.q0*q.q2));
    rot.yaw = atan2f(q.q1*q.q2 + q.q0*q.q3, 0.5f - q.q2*q.q2 - q.q3*q.q3);
    return rot;
}
