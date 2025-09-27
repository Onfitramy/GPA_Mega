#include "calibration_data.h"

#include <stdbool.h>

const CalibrationData_t CalibrationData[5][3] = {
    // GPA Mega 1 NOT CALIBRATED YET
    {
        {{0, 0, 0}, {1, 1, 1}}, // IMU1
        {{0, 0, 0}, {1, 1, 1}}, // IMU2
        {{0, 0, 0}, {1, 1, 1}}, // MAG
    },
    // GPA Mega 2 NOT CALIBRATED YET
    {
        {{0, 0, 0}, {1, 1, 1}}, // IMU1
        {{0, 0, 0}, {1, 1, 1}}, // IMU2
        {{0, 0, 0}, {1, 1, 1}}, // MAG
    },
    // GPA Mega 3 IMU2 not calibrated yet
    {
        {{0.0475, -0.1725, -0.04}, {1.0059, 0.9932, 0.997}}, // IMU1
        {{0, 0, 0}, {1, 1, 1}}, // IMU2
        {{0.6186, -0.1259, -0.1587}, {1.18, 1.14, 1.32}} // MAG
    },
    // GPA Mega 4 IMU2 not calibrated yet
    {
        {{-0.005, -0.085, 0.17}, {0.9954, 0.9944, 0.9959}}, // IMU1
        {{0, 0, 0}, {1, 1, 1}}, // IMU2
        {{0.285, -0.0361, -0.455}, {1.15, 1.1094, 1.28}}, // MAG

    },
    // GPA Mega 5 NOT CALIBRATED YET
    {
        {{0, 0, 0}, {1, 1, 1}}, // IMU1
        {{0, 0, 0}, {1, 1, 1}}, // IMU2
        {{0, 0, 0}, {1, 1, 1}}, // MAG
    }
};

static const uint32_t GPA_MegaUIDs[5][3] = {
    /// GPA Mega 1
    {2883642, 892489994, 842609714},
    // GPA Mega 2 not determined yet
    {0, 0, 0},
    // GPA Mega 3 not determined yet
    {0, 0, 0},
    /// GPA Mega 4
    {2949174, 842223877, 825439797},
    /// GPA Mega 5
    {2949188, 842223877, 825439797},
};

static bool UID_matches(uint32_t uid[3], GPA_Mega gpa_mega) {
    bool matches = true;

    for (int i = 0; i < 3; ++i) {
        uint32_t uid_element = uid[i];
        uint32_t gpa_mega_element = GPA_MegaUIDs[gpa_mega][i];

        if (uid_element != gpa_mega_element) {
            matches = false;
        }
    }

    return matches;
}

GPA_Mega GPA_MegaFromUID(uint32_t uid[3]) {
    if (UID_matches(uid, GPA_MEGA_1))
        return GPA_MEGA_1;
    if (UID_matches(uid, GPA_MEGA_2))
        return GPA_MEGA_2;
    if (UID_matches(uid, GPA_MEGA_3))
        return GPA_MEGA_3;
    if (UID_matches(uid, GPA_MEGA_4))
        return GPA_MEGA_4;
    if (UID_matches(uid, GPA_MEGA_5))
        return GPA_MEGA_5;
    // TODO: some kind of error handling
}
