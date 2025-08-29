Due to limited time availability, I was able to complete Task 1: Decoding GPS data (in UBX format) from the u-blox receiver

Task 1 Summary – Decoding GPS Data (UBX Format)

Task Statement:
Decode GPS data (in UBX format) from a u-blox receiver and extract relevant navigation data for use in the planner.

1. Understanding the Problem:
I began with no prior knowledge of UBX. With AI guidance, I learned that UBX is a binary protocol from u-blox GPS modules, where NAV-POSLLH messages contain latitude, longitude, and altitude as signed 32-bit integers (degrees × 1e7) stored in little endian format.

2. Setting Up the Codebase:
The provided project had multiple .cpp files, but was missing headers. AI helped me reconstruct these, fix compile errors (like M_PI not defined), and understand each module’s role:

ublox_reader → decoding UBX messages

gridmap → mapping GPS to grid coordinates

planning → pathfinding (A*)

odometry → motion commands

main → overall workflow

3. Decoding UBX Messages

At first, the program read only “Raw UBX” lines. By correcting byte offsets and ensuring little-endian interpretation, the code began outputting correct GPS coordinates. Since no real GPS log was available, I generated test UBX hex data for Bangalore coordinates and stored them in gps_data.txt.

4. Grid Mapping & Path Planning

Initially, the grid was too small (10×10, 1m per cell), so the goal was outside the map. I expanded it to 20×20 with 200m cells, which fit both start and goal. The original planner was incomplete, so with AI’s help, I implemented A* search, which produced a valid path around obstacles.

5. Odometry

The odometry module was adjusted to match the header and compute total travel time and turning angle. This output is written to odom_output.txt.
