#ifndef UBLOX_READER_H
#define UBLOX_READER_H

#include <string>
#include <utility>
#include <cstdint>   // <-- this is what was missing!

// UBX NAV-POSLLH fields
struct classId {
    int32_t iTOW;     // GPS time of week (ms)
    int32_t lon;      // Longitude (1e-7 deg)
    int32_t lat;      // Latitude (1e-7 deg)
    int32_t height;   // Height above ellipsoid (mm)
    int32_t hMSL;     // Height above mean sea level (mm)
    uint32_t hAcc;    // Horizontal accuracy (mm)
    uint32_t vAcc;    // Vertical accuracy (mm)
};

// Simple struct for decoded GPS coordinates
struct GPS {
    double lat;
    double lon;
    double height;
};

// Function prototype
std::pair<GPS, GPS> readUbloxFile(const std::string &filename);

#endif
