/* ===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=============================================== */

/*----------------------------- Imports ------------------------------*/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

/*---------------------------- Data Types ----------------------------*/
typedef byte C; // Aliases bytes as coordinates to help distinguish between walls and coordinates, both of which are stored as bytes

/*--------------------------- Declarations ---------------------------*/
// Pins
const int trig[3] = {2, 4, 6};
const int echo[3] = {3, 5, 7};
const int forwardL = 8;
const int reverseL = 9;
const int enL = 10;
const int enR = 11;
const int forwardR = 12;
const int reverseR = 13;
const int interrupt_pin = A0;
// SDA -> A4
// SCL -> A5

// Variables
MPU6050 mpu;
bool dmpReady = false;              // set true if DMP init was successful
uint8_t mpuIntStatus;               // holds actual interrupt status byte from MPU
uint8_t devStatus;                  // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                 // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];             // FIFO storage buffer
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
bool blinkstate = false;
byte walls[16][16];
int orient = 0;
C pos = 0x00;
C next_pos;
const C destination[4] = {0x77, 0x78, 0x87, 0x88};
float D[3];
Quaternion q;        // [w, x, y, z] quaternion container
VectorFloat gravity; // vector storing the direction and magnitude of gravitational acceleration
float ypr[3];        // yaw, pitch and roll array (yaw is the only value we care about)
float yaw;
const float square_width = 15; // exact width of each grid square in cm
const float wheel_distance = 1; // exact distance between the left wheels and right wheels in cm
const byte turn_speed = 200; // 255 is the regular speed

/*-------------------------- Input Modules ---------------------------*/
void scan(int direction) {

  // A clean HIGH pulse is sent to the chosen ultrasonic sensor, triggering it to emit sound
  digitalWrite(trig[direction], LOW);
  delayMicroseconds(5);
  digitalWrite(trig[direction], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig[direction], LOW);

  // Calculates distance in squares by recording the time it takes for the sound to echo back to the sensor
  D[direction] = pulseIn(echo[direction], HIGH) * 0.01715 / square_width;
}

// Callback function for interrupt management from mpu
void dmpDataReady() {
  mpuInterrupt = true;
}

void get_yaw() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  yaw = ypr[0];
}

void update_walls() {
  for (int i = 0; i < 3; i++) {scan(i);}
  get_yaw();

  // Calculates distance to each wall
  float cos_phi = cos(yaw - orient * HALF_PI);
  int Sw, Sn, Se, n, e, s, w, temp;
  const int y = pos >> 4;
  const int x = pos & 0x0F;
  cos_phi = cos(yaw - orient * HALF_PI);
  Sw = D[0] * cos_phi;
  Sn = D[1] * cos_phi;
  Se = D[2] * cos_phi;

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

/*------------------------ Processing Modules ------------------------*/
bool in(C coord, C* coords, int size) {
  for (int i = 0; i < size; ++i) {
    if (coord == coords[i]) {
      return true;
    }
  }
  return false;
}

// Calculates the fastest path from the mouse to the destination based on the information found by the mouse, returns the first coordinate on that path
void floodfill() { 
  
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
            next_pos = c;
            return;
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

/*-------------------------- Output Modules --------------------------*/
void power_motors(bool dirL, byte powerL, bool dirR, byte powerR) {
  
  // Powers left motor
  analogWrite(enL, powerL);
  if (dirL) {
    digitalWrite(forwardL, HIGH);
    digitalWrite(reverseL, LOW);
  } else {
    digitalWrite(forwardL, LOW);
    digitalWrite(reverseL, HIGH);
  }

  // Powers right motor
  analogWrite(enR, powerR);
  if (dirR) {
    digitalWrite(forwardR, HIGH);
    digitalWrite(reverseR, LOW);
  } else {
    digitalWrite(forwardR, LOW);
    digitalWrite(reverseR, HIGH);
  }
}

void reorient() {
  get_yaw();

  const C old_pos = pos;
  pos = next_pos;

  // Orientate north
  if ((old_pos + 0x10 == next_pos) && (yaw > 0.1) && (yaw < 2 * PI - 0.1)) {
    orient = 0;
    if (yaw < PI) {
      power_motors(0, turn_speed, 1, turn_speed);
      while (yaw < PI) {
        delay(10);
        get_yaw();
      }
    } else {
      power_motors(1, turn_speed, 0, turn_speed);
      while (yaw > PI) {
        delay(10);
        get_yaw();
      }
    }

  // Orientate east
  } else if ((old_pos + 0x01 == next_pos) && (abs(yaw - HALF_PI) > 0.1)) {
    orient = 1;
    if (yaw < HALF_PI || yaw > 3 * HALF_PI) {
      power_motors(0, turn_speed, 1, turn_speed);
      while (yaw < HALF_PI || yaw > 3 * HALF_PI) {
        delay(10);
        get_yaw();
      }
    } else {
      power_motors(1, turn_speed, 0, turn_speed);
      while ((yaw > HALF_PI) && (yaw < 3 * HALF_PI)) {
        delay(10);
        get_yaw();
      }
    }

  // Orientate south
  } else if ((old_pos - 0x10 == next_pos) && (abs(yaw - PI) > 0.1)) {
    orient = 2;
    if (yaw > PI) {
      power_motors(0, turn_speed, 1, turn_speed);
      while (yaw > PI) {
        delay(10);
        get_yaw();
      }
    } else {
      power_motors(1, turn_speed, 0, turn_speed);
      while (yaw < PI) {
        delay(10);
        get_yaw();
      }
    }

  // Orientate west
  } else if (abs(yaw - HALF_PI * 3) > 0.1) {
    orient = 3;
    if (yaw > HALF_PI && yaw < 3 * HALF_PI) {
      power_motors(0, turn_speed, 1, turn_speed);
      while (yaw > HALF_PI && yaw < 3 * HALF_PI) {
        delay(10);
        get_yaw();
      }
    } else {
      power_motors(1, turn_speed, 0, turn_speed);
      while (yaw < HALF_PI || yaw > 3 * HALF_PI) {
        delay(10);
        get_yaw();
      }
    }
  }

  power_motors(1, 0xFF, 1, 0xFF);
}

// Places the mouse ot the centre of the next square
void centre() {
  for (int i = 0; i < 3; i++) {scan(i);}
  get_yaw();

  // Calculates distances to each wall
  float phi, cos_phi, tempw, tempn, tempe, Sw, Sn, Se, h, projected;
  phi = yaw - orient * HALF_PI;
  cos_phi = cos(phi);
  tempw = D[0] * cos_phi;
  tempn = D[1] * cos_phi;
  tempe = D[2] * cos_phi;
  Sw = tempw - floor(tempw);
  Sn = tempn - floor(tempn);
  Se = tempe - floor(tempe);
  h = Sn + 0.5;
  projected = h * tan(phi) + Sw - Se;

  // Brings the mouse directly to the centre of the next square along a straight line
  if (abs(projected) < 0.1) {
    power_motors(1, 0xFF, 1, 0xFF);
    if (Sn > 0.5) {
      while (Sn > 0.4) {
        delay(10);
        scan(1);
        tempn = D[1] * cos_phi;
        Sn = tempn - floor(tempn);
      }
    }
    while (Sn < 0.5) {
      delay(10);
      scan(1);
      tempn = D[1] * cos_phi;
      Sn = tempn - floor(tempn);
    }
    while (Sn > 0.5) {
      delay(10);
      scan(1);
      tempn = D[1] * cos_phi;
      Sn = tempn - floor(tempn);
    }
  }

  else {
    float w, s, theta, power_difference;
    w = abs(Sw - Se);
    s = sqrt(h*h + w*w);
    theta = acos((h*cos_phi + h - w*sin(yaw))/s) + acos(-h/s) + PI;
    if (theta > 2 * PI) theta -= 2 * PI;
    power_difference = turn_speed * wheel_distance * (1 + cos_phi + 2 * cos(theta)) / (2 * w);

    // Course-corrects the mouse if its going too far right
    if (projected > 0) {

      // Differentially drives slightly leftwards
      power_motors(1, round(turn_speed - power_difference), 1, round(turn_speed + power_difference));
      while (phi > theta || phi < PI) {
        delay(10);
        get_yaw();
      }

      // Differentially drives rightwards by the same degree but for a different length of time
      power_motors(1, round(turn_speed + power_difference), 1, round(turn_speed - power_difference));
      while (phi > PI) {
        delay(10);
        get_yaw();
      }
    }

    // Course-corrects the mouse if its going too far left
    else {

      // Differentially drives slightly rightwards
      power_motors(1, round(turn_speed + power_difference), 1, round(turn_speed - power_difference));
      while (phi < theta || phi > PI) {
        delay(10);
        get_yaw();
        phi = yaw - orient * HALF_PI;
      }

      // Differentially drives leftwards by the same degree but for a different length of time
      power_motors(1, round(turn_speed - power_difference), 1, round(turn_speed + power_difference));
      while (phi < PI) {
        delay(10);
        get_yaw();
        phi = yaw - orient * HALF_PI;
      }
    }

    // Returns mouse to a straight path
    power_motors(1, 0xFF, 1, 0xFF);
  }
}

/*------------------------------- Main -------------------------------*/
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialises variables
  for (int y = 0; y < 16; ++y) {
    for (int x = 0; x < 16; ++x) {
      walls[y][x] = 0x00;
    }
  }

  // Inputs and outputs
  pinMode(interrupt_pin, INPUT);
  for (int i = 0; i < 3; ++i) {
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  for (int i = 8; i < 14; ++i) {
    pinMode(i, OUTPUT);
  }
  digitalWrite(forwardL, LOW);
  digitalWrite(reverseL, LOW);
  digitalWrite(forwardR, LOW);
  digitalWrite(reverseR, LOW);

  // Initialises gyroscope
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  update_walls();
  floodfill();
  reorient();
  centre();
  if (in(pos, destination, 4)) return;
}
