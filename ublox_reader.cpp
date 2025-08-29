#include "ublox_reader.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// Decode NAV-POSLLH payload (28 bytes)
static int NAV_POSLLH(uint8_t *buffer, classId *gps) {
    memcpy(&gps->iTOW,   buffer + 0,  4);
    memcpy(&gps->lon,    buffer + 4,  4);
    memcpy(&gps->lat,    buffer + 8,  4);
    memcpy(&gps->height, buffer + 12, 4);
    memcpy(&gps->hMSL,   buffer + 16, 4);
    memcpy(&gps->hAcc,   buffer + 20, 4);
    memcpy(&gps->vAcc,   buffer + 24, 4);
    return 0;
}

// Convert "01 02 AC 4A ..." string into bytes
static vector<uint8_t> hexToBytes(const string &rawHex) {
    vector<uint8_t> bytes;
    stringstream ss(rawHex);
    string token;
    while (ss >> token) {
        bytes.push_back(static_cast<uint8_t>(stoul(token, nullptr, 16)));
    }
    return bytes;
}

// Decode UBX message (expects buffer[0] = class, buffer[1] = ID)
static int decodeUBX(uint8_t *buffer, classId *gps) {
    if (buffer[0] == 0x01 && buffer[1] == 0x02) { // NAV-POSLLH
        // Skip class (1) + id (1) → payload starts at buffer+2
        return NAV_POSLLH(buffer + 2, gps);
    }
    return 1;
}

// Convert from raw UBX struct to human-readable GPS
static GPS gpsFromData(const classId &gps) {
    GPS out;
    out.lat = gps.lat * 1e-7;
    out.lon = gps.lon * 1e-7;
    out.height = gps.height / 1000.0; // mm -> m
    return out;
}

// Read file with 2 UBX hex lines (start & goal)
pair<GPS, GPS> readUbloxFile(const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: cannot open file " << filename << endl;
        return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    }

    string rawStart, rawGoal;
    getline(file, rawStart);
    getline(file, rawGoal);

    cout << "Raw UBX Start: " << rawStart << endl;
    cout << "Raw UBX Goal : " << rawGoal << endl;

    vector<uint8_t> startBytes = hexToBytes(rawStart);
    vector<uint8_t> goalBytes  = hexToBytes(rawGoal);

    classId gpsStartData{}, gpsGoalData{};
    decodeUBX(startBytes.data(), &gpsStartData);
    decodeUBX(goalBytes.data(), &gpsGoalData);

    GPS startGPS = gpsFromData(gpsStartData);
    GPS goalGPS  = gpsFromData(gpsGoalData);

    file.close();
    return {startGPS, goalGPS};
}
