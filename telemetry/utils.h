#ifndef UTILS_H
#define UTILS_H

#include <chrono>

namespace Utils {
    double degreesToRadians(double degrees);
    double radiansToDegrees(double radians);
    long long getCurrentTimestampMs(); // Milliseconds since epoch
}

#endif // UTILS_H