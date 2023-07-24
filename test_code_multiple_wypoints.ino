void initGPS() {
  Serial1.begin(GPSBAUD);
  Serial.println("Waiting for GPS signal...");
}

void getgps(TinyGPS &gps) {
  float gpsX, gpsY;
  gps.f_get_position(&gpsX, &gpsY);
  X = (double)gpsX;
  Y = (double)gpsY;
}

void initCompass() {
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

void initmotorPID() {
  motorInput = theta;
  motorSetpoint = alpha;
  motorPID.SetMode(AUTOMATIC);
}

void initpropellerPID() {
  propellerInput = sqrt(pow(X, 2) + pow(Y, 2));
  propellerSetpoint = sqrt(pow(waypointLat, 2) + pow(waypointLon, 2));
  //propellerInput=distance;
  //propellerSetpoint=1;
  propellerPID.SetMode(AUTOMATIC);
}

void currentPos() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (gps.encode(c)) {
      getgps(gps);
    }
  }
}

double distanceBetween(double lat1, double long1, double lat2, double long2) {
  // Returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for the hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

void getDistanceWaypoint() {
  distance = sqrt(pow((X - waypointLat), 2) + pow((Y - waypointLon), 2)) * RAD;

  if (DEBUG_MODE) {
    Serial.print("\nDistance to waypoint: ");
    Serial.println(distance, 6);
  }
}

void currentHead() {
  int x, y, z; // Triple axis data

  Wire.beginTransmission(address);
  Wire.write(0x03); // Select register 3, X MSB register
  Wire.endTransmission();

  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; // X MSB
    x |= Wire.read(); // X LSB
    z = Wire.read() << 8; // Z MSB
    z |= Wire.read(); // Z LSB
    y = Wire.read() << 8; // Y MSB
    y |= Wire.read(); // Y LSB
  }

  theta = (atan2(-y, x) / 3.141 * 180) + 90;
  if (theta < 0) {
    theta = 360 + theta;
  }

  if (DEBUG_MODE) {
    Serial.println("\nTheta ");
    Serial.print(theta);
    Serial.print(" X: ");
    Serial.print(x);
    Serial.print("  Y: ");
    Serial.print(y);
    Serial.print("  Z: ");
    Serial.println(z);
    cmp_x = x;
    cmp_y = y;
    cmp_z = z;
  }
}

void getAlpha() {
  alpha = atan2((X - waypointLat), (waypointLon - Y));
  alpha = (360 / (2 * pi)) * alpha;
  alpha = alpha + 90; // Measure from north.

  if (alpha < 0) {
    alpha = alpha + 360; // Make to 0 - 360 range.
  }
}

void getBeta() {
  beta = alpha - theta;
}

void pidMotor() {
  motorPID.Compute();
}

void pidPropeller() {
  propellerPID.Compute();
  propellerSpeed = map(propellerOutput, 0, 255, 50, 150);
  //Serial.print("Propeller Output");

  if (distance <= 1) {
    propellerSpeed = 0;
  }
}

void display1() {
  unsigned long now = millis();
  unsigned long timeChange = now - lastDispTime;

  if (timeChange > 1000) {
    // Serial.print("distance to waypoint:");
    // Serial.println(distance);
    // Serial.print("Alpha:");
    // Serial.println(alpha, 5);
    // Serial.print("Beta:");
    // Serial.println(beta, 5);
    lastDispTime = millis();
  }
}

/* Please note that the provided code may still need further testing and adjustments depending on your specific application and hardware setup. 
Always test the code thoroughly and ensure that it meets your requirements before deploying it in a real-world environment. */
