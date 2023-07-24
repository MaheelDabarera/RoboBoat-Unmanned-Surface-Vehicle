#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

// Function prototypes
double distance_between(double lat1, double long1, double lat2, double long2);
void get_alpha(double X, double Y, double waypoint_lat, double waypoint_lon, double& alpha);
void get_beta(double alpha, double theta, double& beta);
void go_to_waypoint(double X, double Y, double theta, double waypoint_lat, double waypoint_lon);
void navigate_to_waypoints(const vector<pair<double, double>>& waypoints);

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

  navigate_to_waypoints(waypoints);

  cout << "Reached all waypoints." << endl;

  return 0;
}

double distance_between(double lat1, double long1, double lat2, double long2) {
  double delta_lat = radians(lat2 - lat1);
  double delta_long = radians(long2 - long1);

  double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
           cos(radians(lat1)) * cos(radians(lat2)) *
           sin(delta_long / 2) * sin(delta_long / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS * c;
}

void get_alpha(double X, double Y, double waypoint_lat, double waypoint_lon, double& alpha) {
  alpha = atan2((X - waypoint_lat), (waypoint_lon - Y));
  alpha = (360 / (2 * pi)) * alpha;
  alpha = alpha + 90; // Measure from north.

  if (alpha < 0) alpha += 360; // Make it in the 0 - 360 range.
}

void get_beta(double alpha, double theta, double& beta) {
  beta = alpha - theta;
}

void go_to_waypoint(double X, double Y, double theta, double waypoint_lat, double waypoint_lon) {
  double alpha, beta, distance;

  do {
    // Calculate alpha and beta
    get_alpha(X, Y, waypoint_lat, waypoint_lon, alpha);
    get_beta(alpha, theta, beta);

    // Compute PID output and adjust motor speed
    double motor_speed = MOTOR_SPEED_BASE + MOTOR_PID_KP * beta;
    motor_speed = max(motor_speed, MOTOR_SPEED_MIN);
    motor_speed = min(motor_speed, MOTOR_SPEED_MAX);

    // Update boat's position and heading (You need to implement these functions)
    // ... (Update X, Y, and theta based on boat's GPS and compass data)

    distance = distance_between(X, Y, waypoint_lat, waypoint_lon);
  } while (distance >= WAYPOINT_REACHED_THRESHOLD);

  // Stop the boat when the waypoint is reached
  // ... (Add boat stopping logic here)
