#include "gridmap.h"
#include "odometry.h"
#include "planning.h"
#include "ublox_reader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

// Define M_PI if not available (Windows/MSVC often misses it)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// Helper to convert angle to unit direction
pair<double, double> directionFromAngle(double angle_deg) {
    double rad = angle_deg * M_PI / 180.0;
    return {cos(rad), sin(rad)};
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <gps_data_file> <output_file>" << endl;
        return 1;
    }

    // store file path to GPS data
    string gps_data = argv[1];

    // file path to store result
    string odom_commands = argv[2];

    // decode GPS data from file
    auto result = readUbloxFile(gps_data);
    if (static_cast<int>(result.first.lat) == 0 &&
        static_cast<int>(result.first.lon) == 0 &&
        static_cast<int>(result.second.lat) == 0 &&
        static_cast<int>(result.second.lon) == 0) {
        cout << "Error: Invalid GPS Coordinates" << endl;
        return 1;
    }

    cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon << endl;
    cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon << endl;

    // Initialize Gridmapper with start as origin
    GPS origin = {result.first.lat, result.first.lon};

    // Make grid coarse enough to cover km-scale GPS differences
    double cellsize = 200.0; // each grid cell = 200 meters
    int rows = 20, cols = 20; // 20x20 grid (~4 km x 4 km)
    Gridmapper grid(origin, cellsize, rows, cols);

    // Convert start and goal GPS to grid coordinates
    pair<int, int> start = grid.gpstogrid(result.first);
    pair<int, int> goal = grid.gpstogrid(result.second);

    cout << "Start (grid) -> (" << start.first << "," << start.second << ")" << endl;
    cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")" << endl;

    // Path planning
    Planner planner(grid.getGrid());
    auto path = planner.pathplanning(start, goal);

    // print planned path
    cout << "Planned Path:" << endl;
    if (path.empty()) {
        cout << "No path found!" << endl;
    } else {
        for (auto &p : path) {
            cout << "(" << p.first << "," << p.second << ")" << endl;
        }
    }

    // Odometry commands
    cout << "\nOdometry Commands" << endl;
    double wheel_radius = 0.05; // meters
    double rpm = 120;           // wheel speed
    Odometry odo(wheel_radius, rpm);
    auto commands = odo.computeCommands(path);

    // computing total time and sec
    ofstream result_file(odom_commands);

    // check if file is open
    if (!result_file.is_open()) {
        cerr << "Error: cannot open file " << odom_commands << endl;
        return 1;
    }

    // writing result to file
    result_file << commands.time_sec << endl << commands.angle_deg << endl;

    // closing file
    result_file.close();

    return 0;
}
