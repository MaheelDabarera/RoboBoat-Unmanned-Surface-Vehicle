#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

// Function prototypes
double distanceBetween(double lat1, double long1, double lat2, double long2);
void getAlpha(double X, double Y, double waypointLat, double waypointLon, double& alpha);
void getBeta(double alpha, double theta, double& beta);
void goToWaypoint(double X, double Y, double theta, double waypointLat, double waypointLon);
void navigateToWaypoints(const vector<pair<double, double>>& waypoints);

// Constants
const double pi = 3.141592;
const double EARTH_RADIUS = 6372795; // in meters
const double WAYPOINT_REACHED_THRESHOLD = 0.5; // in meters
const double ANGLE_THRESHOLD = 0.1; // in degrees
const double MOTOR_SPEED_BASE = 30; // Base motor speed
const double MOTOR_SPEED_MIN = 0;
const double MOTOR_SPEED_MAX = 50;
const double MOTOR_PID_KP = 1;
const double MOTOR_PID_KI = 0;
const double MOTOR_PID_KD = 0;

int main() {
    vector<pair<double, double>> waypoints = {
        {14.078877, 100.614978}, // First waypoint
        {14.078877, 100.614978}  // Change second waypoint
    };

    navigateToWaypoints(waypoints);

    cout << "Reached all waypoints." << endl;

    return 0;
}

double distanceBetween(double lat1, double long1, double lat2, double long2) {
    double deltaLat = radians(lat2 - lat1);
    double deltaLong = radians(long2 - long1);

    double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
               cos(radians(lat1)) * cos(radians(lat2)) *
               sin(deltaLong / 2) * sin(deltaLong / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c;
}

void getAlpha(double X, double Y, double waypointLat, double waypointLon, double& alpha) {
    alpha = atan2((X - waypointLat), (waypointLon - Y));
    alpha = (360 / (2 * pi)) * alpha;
    alpha = alpha + 90; // Measure from north.

    if (alpha < 0) alpha += 360; // Make it in the 0 - 360 range.
}

void getBeta(double alpha, double theta, double& beta) {
    beta = alpha - theta;
}

void goToWaypoint(double X, double Y, double theta, double waypointLat, double waypointLon) {
    double alpha, beta, distance;

    do {
        // Calculate alpha and beta
        getAlpha(X, Y, waypointLat, waypointLon, alpha);
        getBeta(alpha, theta, beta);

        // Compute PID output and adjust motor speed
        // ... (Add PID and motor control logic here)

        // Calculate new distance to the waypoint
        distance = distanceBetween(X, Y, waypointLat, waypointLon);

        // Update boat's position and heading (You need to implement these functions)
        // ... (Update X, Y, and theta based on boat's GPS and compass data)

    } while (distance >= WAYPOINT_REACHED_THRESHOLD);

    // Stop the boat when the waypoint is reached
    // ... (Add boat stopping logic here)
}

void navigateToWaypoints(const vector<pair<double, double>>& waypoints) {
    for (const auto& waypoint : waypoints) {
        double waypointLat = waypoint.first;
        double waypointLon = waypoint.second;

        // Wait until GPS is locking (You need to implement this function)
        // ... (Wait until GPS data is available)

        while (true) {
            // Update boat's position and heading (You need to implement these functions)
            // ... (Update X, Y, and theta based on boat's GPS and compass data)

            // Go to the current waypoint
            goToWaypoint(X, Y, theta, waypointLat, waypointLon);

            // Check if the boat is near the waypoint
            double distance = distanceBetween(X, Y, waypointLat, waypointLon);
            if (distance < WAYPOINT_REACHED_THRESHOLD) {
                break; // Move to the next waypoint
            }

            // Add a small delay between iterations (You can adjust this delay as needed)
            // ... (Add delay here if needed)
        }
    }
}
