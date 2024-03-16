/*----------------------------- Imports ------------------------------*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*---------------------------- Data Types ----------------------------*/
// Aliases bytes as coordinates to help distinguish between walls and coordinates, both of which are stored as bytes
typedef byte C;

/*--------------------------- Declarations ---------------------------*/
// Variables
byte walls[16][16];
int orient = 0;
C pos = 0x00;
float Ax, Ay, Gz, Dw, Dn, De;
sensors_event_t Acc, Gyro, Temp;

// Constants
const int MOTOR_OUTPUT, trigPin, echoPin;
const float PI = 3.141593;
const float HALF_PI = 1.570796;
trigPin = 3;
echoPin = 2;

/*---------------------------- Functions -----------------------------*/
// Checks whether a coordinate is in an array of coordinates
bool in(C coord, C* coords, int size) {
  for (int i = 0; i < size; ++i) {
    if (coord == coords[i]) {
      return true;
    }
  }
  return false;
}

// Roughly rotates the mouse to allign with a given orientation
void reorient(C new_pos) {
  // Check gyroscope

  const C old_pos = pos;
  pos = new_pos;

  // Orientate north
  if (old_pos + 0x10 == new_pos) {
    if ((orient == 0) && (Gz < 0.1 || Gz > 2 * PI - 0.1)) return;
    bool disoriented() {return Gz < PI};
    orient = 0;

  // Orientate east
  } else if (old_pos + 0x01 == new_pos) {
    if ((orient == 1) && (abs(Gz - HALF_PI) < 0.1)) return;
    bool disoriented() {return Gz < HALF_PI || Gz > 3 * HALF_PI};
    orient = 1;

  // Orientate south
  } else if (old_pos - 0x10 == new_pos) {
    if ((orient == 2) && (abs(Gz - PI) < 0.1)) return;
    bool disoriented() {return Gz > PI};
    orient = 2;

  // Orientate west
  } else {
    if ((orient == 3) && (abs(Gz - HALF_PI * 3) < 0.1)) return;
    bool disoriented() {Gz > HALF PI && Gz < 3 * HALF_PI};
    orient = 3;
  }

  // Rotates clockwise or anticlockwise until the orientation is a certain value
  if (disoriented()) {
    // Reverse left motor
    // Activate motors
    while (disoriented()) {
      delay(10)
      // Check gyroscope
    }
    // Reverse left motor
  } else {
    // Reverse right motor
    // Activate motors
    while (!disoriented()) {
      delay(10)
      // Check gyroscope
    }
    // Reverse right motor
  }
}

// Places the mouse ot the centre of the next square
void centre() {
  // Check all sensors

  // Calculates distances to each wall
  float phi, tempn, Sn;
  const float cos_phi, h, projected, tempw, Sw, tempe, Se;
  phi = Gz - orient * HALF_PI;
  cos_phi = cos(phi);
  tempn = Dn * cos_phi;
  Sn = tempn - floor(tempn)
  h = Sn + 0.5;
  tempw = Dw * cos_phi;
  Sw = tempw - floor(tempw);
  tempe = De * cos_phi;
  Se = tempe - floor(tempe);
  projected = h * tan(phi) + Sw - Se;

  // Course-corrects the mouse if its going too far right
  if (projected > 0.1) {
    const float w, s, theta, inv_r;
    w = abs(Sw - Se);
    s = sqrt(h**2 + w**2);
    theta = acos((h*cos_phi + h - w*sin(Gz))/s) + acos(-h/s) + PI;
    if (theta > 2 * PI) theta -= 2 * PI;
    inv_r = (1 + cos_phi + 2 * cos(theta)) / w;

    // Output MOTOR_OUTPUT to right motor
    // Output MOTOR_OUTPUT - inv_r to left motor
    while (phi > theta || phi < PI) {
      delay(10);
      // Check gyroscope
    }
    // Output MOTOR_OUTPUT to left motor
    // Output MOTOR_OUTPUT - inv_R to right motor
    while (phi > PI) {
      delay(10);
      // Check gyroscope
    }
    // Output MOTOR_OUTPUT to right motor
  }

  // Course-corrects the mouse if its going too far left
  else if (projected < -0.1) {
    const float w, s, theta, inv_r;
    w = abs(Sw - Se);
    s = sqrt(h**2 + w**2);
    theta = acos((h*cos_phi + h - w*sin(Gz))/s) + acos(-h/s) + PI;
    if (theta > 2 * PI) theta -= 2 * PI;
    inv_r = (1 + cos_phi + 2 * cos(theta)) / w;

    // Output MOTOR_OUTPUT to left motor
    // Output MOTOR_OUTPUT - inv_r to right motor
    while (phi < theta || phi > PI) {
      delay(10);
      // Check gyroscope
      phi = Gz - orient * HALF_PI;
    }
    // Output MOTOR_OUTPUT to right motor
    // Output MOTOR_OUTPUT - inv_R to left motor
    while (phi < PI) {
      delay(10);
      // Check gyroscope
      phi = Gz - orient * HALF_PI;
    }
    // Output MOTOR_OUTPUT to right motor
  }

  // Brings the mouse directly to the centre of the next square along a straight line
  else {
    // Output MOTOR_OUTPUT to right motor
    // Output MOTOR_OUTPUT to left motor
    if (Sn > 0.5) {
      while (Sn > 0.4) {
        delay(10);
        // Check north ultrasonic sensor
        tempn = Dn * cos_phi;
        Sn = tempn - floor(tempn);
      }
    }
    while (Sn < 0.5) {
      delay(10);
      // Check north ultrasonic sensor
      tempn = Dn * cos_phi;
      Sn = tempn - floor(tempn);
    }
    while (Sn > 0.5) {
      delay(10);
      // Check north ultrasonic senor
      tempn = Dn * cos_phi;
      Sn = tempn - floor(tempn);
    }
  }
}

void update_walls() {
  // Check all sensors

  // Calculates distance to each wall
  const float cos_phi;
  const int Sw, Sn, Se, n, e, s, w;
  const int y = pos >> 4;
  const int x = pos & 0x0F;
  int temp;
  cos_phi = cos(Gz - orient * HALF_PI);
  Sw = Dw * cos_phi;
  Sn = Dn * cos_phi;
  Se = De * cos_phi;

  // Updates walls array accordingly
  if (orient == 0) {
    temp = y + Sn;
    walls[temp][x] /= 0x08;
    if (temp != 15) walls[temp + 1][x] /= 0x02;

    temp = x - Sw;
    walls[y][temp] /= 0x01;
    if (temp != 0) walls[y][temp - 1] /= 0x04;

    temp = x + Se;
    walls[y][temp] /= 0x04;
    if (temp != 15) walls[y][temp + 1] /= 0x01;
  } 
  else if (orient == 1) {
    temp = x + Sn;
    walls[y][temp] /= 0x04;
    if (temp != 15) walls[y][temp + 1] /= 0x01;

    temp = y - Se;
    walls[temp][x] /= 0x02;
    if (temp != 0) walls[temp - 1][x] /= 0x08;

    temp = y + Sw;
    walls[temp][x] /= 0x08;
    if (temp != 15) walls[temp + 1][x] /= 0x02;
  }
  else if (orient == 2) {
    temp = y - Sn;
    walls[temp][x] /= 0x02;
    if (temp != 0) walls[temp - 1][x] /= 0x08;

    temp = x - Se;
    walls[y][temp] /= 0x01;
    if (temp != 0) walls[y][temp - 1] /= 0x04;

    temp = x + Sw;
    walls[y][temp] /= 0x04;
    if (temp != 15) walls[y][temp + 1] /= 0x01;
  }
  else {
    temp = x - Sn;
    walls[y][temp] /= 0x01;
    if (temp != 0) walls[y][temp - 1] /= 0x04;

    temp = y - Sw;
    walls[temp][x] /= 0x02;
    if (temp != 0) walls[temp - 1][x] /= 0x08;

    temp = y + Se;
    walls[temp][x] /= 0x08;
    if (temp != 15) walls[temp + 1][x] /= 0x02;
  }
}

// Calculates the fastest path from the mouse to the destination based on the information found by the mouse, returns the first coordinate on that path
C floodfill(C pos) { 
  
  // Declares the necessary arrays for the floodfill calculation
  unsigned int source_len = 4;
  unsigned int prev_len = 0;
  C* source = new C[32];
  C* prev_source = new C[32];
  source[0] = 0x77;
  source[1] = 0x78;
  source[2] = 0x87;
  source[3] = 0x88;

  // Initialises a breadth-first search through the maze till the mouse has been reached, returning the parent node to the mouse's node as it is the coordinate the mouse must move to
  while (true) {
    unsigned int sink_idx = 0;
    C* sink = new C[32];

    // Iterates through the current row of nodes, determining which coordinates form the neighbours of each node are in the next row
    for (int j = 0; j < source_len; ++j) {
      C c = source[j];

      // Checks which of the borders, if any, is neighbouring the current node
      bool not_border[4] = {0, 0, 0, 0};
      if (c << 4) {
        not_border[0] = 1;
      } if (c >> 4) {
        not_border[1] = 1;
      } if ((c + 0x01) << 4) {
        not_border[2] = 1;
      } if ((c + 0x10) >> 4) {
        not_border[3] = 1;
      }

      // Iterates through the horizontally or vertically neighbouring coordinates, checking whether or not there is a known wall between the current node and each neighbour
      const C neighbours[4] = {c - 0x01, c - 0x10, c + 0x01, c + 0x10};
      for (int i = 0; i < 4; ++i) {
        C neighbour = neighbours[i];

        // Checks if there are any known walls between each neighbour and the current source
        if (not_border[i] && !((walls[(int)(neighbour >> 4)][(int)(neighbour & 0x0F)] >> i) & 0x01)) { 
          
          // Returns the current source if the sink reached is the mouse's position
          if (neighbour == pos) {
            return c;
          }

          // Checks if each neighbour has been explored by the source before
          if (!in(neighbour, prev_source, prev_len) && !in(neighbour, sink, sink_idx)) {
            sink[sink_idx++] = neighbour;
          }
        }
      }

      // Reallocates the data in source to prev_source
      prev_source[j] = c;
    }

    // The necessary arrays are prepared for the same operation to occur over the next row of nodes
    prev_len = source_len;
    source_len = sink_idx;
    for (int i = source_len; i < prev_len; ++i) {
      delete &prev_source[i];
    }
    for (int i = 0; i < sink_idx; ++i) {
      source[i] = sink[i];
    }
    for (int i = sink_idx; i < source_len; ++i) {
      delete &source[i];
    }
    delete[] sink;
  }

  // Deallocates the arrays used
  delete[] source;
  delete[] prev_source;
}

/*------------------------------- Main -------------------------------*/
void setup() {
  Serial.begin(9600);

  // Initialises the grid of walls
  for (int i = 0; i < 16; ++i) {
    for (int j = 0; j < 16; ++j) {
      walls[i][j] = B0;
    }
  }
}

void loop() {
  C end[4];
  end[0] = 0x77;
  end[1] = 0x78;
  end[2] = 0x87;
  end[3] = 0x88;

  while (!in(pos, end, 4)) {
    
  }
}
