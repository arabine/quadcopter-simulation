#include "utils.h"
#include <cmath>

namespace Utils {
    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    double radiansToDegrees(double radians) {
        return radians * 180.0 / M_PI;
    }

    long long getCurrentTimestampMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch()
               ).count();
    }
}