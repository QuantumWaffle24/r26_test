#include "gridmap.h"
#include <iostream>

// Define M_PI if not available (Windows/MSVC often misses it)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

Gridmapper::Gridmapper(GPS origin, double cellsize, int rows, int cols)
    : origin(origin), cellsize(cellsize), rows(rows), cols(cols) {
    makemap();
}

pair<int, int> Gridmapper::gpstogrid(const GPS &point) const {
    double dLat = (point.lat - origin.lat) * 111320.0; // approx meters per degree lat
    double dLon = (point.lon - origin.lon) * 111320.0 * cos(origin.lat * M_PI / 180.0);
    int row = static_cast<int>(dLat / cellsize);
    int col = static_cast<int>(dLon / cellsize);
    return {row, col};
}

const vector<vector<bool>> &Gridmapper::getGrid() const {
    return grid;
}

double Gridmapper::deg2rad(double deg) { return deg * M_PI / 180.0; }

bool Gridmapper::isvalid(int row, int col) const {
    return row >= 0 && col >= 0 && row < rows && col < cols;
}

void Gridmapper::makemap() {
    grid = vector<vector<bool>>(rows, vector<bool>(cols, false));
    // add a simple obstacle line
    for (int i = 3; i < 7; i++) {
        grid[i][2] = true;
    }
    for (int j = 4; j < 9; j++) {
        grid[6][j] = true;
    }
    printgrid();
}

void Gridmapper::printgrid() const {
    cout << "The grid map is:" << endl;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            cout << grid[i][j] << " ";
        }
        cout << endl;
    }
}
