/* 
 *  This the program for reading the IMU data and then converting to direction and magnitude
 *  to write by serial to the PC.
 *  It also reads the serial data coming from the PC relating to states and events, and 
 *  converts this to motor PWM states and duration.
 *  Authors: Rob Segal, Steven Holwerda, Puya Abolfathi Visospace Pty Ltd 8/12/17
 *  MPU9250 and Filters library from gitHub
 */
 
/* MPU9250 Breakout --------- Arduino Uno
VDD ---------------------- 3.3V
VDDI --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND
*/

#include <MPU9250.h>
#include <Filters.h>

// IMU declarations
MPU9250 myIMU;
int numLoops = 200;
float axBias;
float ayBias;
float azBias;
float dt;
bool firstRun = true;
float prevTime;
float accel_pitch;
float accel_roll;
float gyro_pitch;
float gyro_roll;
float hpf = 0.95;
float lpf = 1 - hpf;
float combined_pitch;
float combined_roll;
float TestTime;
float TestTimeUpdate = 0;
float xAxis;
float yAxis;
float directionHoVR;
float magnitudeHoVR1;


//Comms Read
const byte numChars = 32;
char receivedChars[numChars];

//Motor state from comms
int motor1 = 5; // pin5
int motor2 = 6; // pin6
int myStr;
int myStr1;
int myStr2;
int myStrLast = 0;
int time_start;
int time_start2;
boolean newData = false;
boolean motorRun = false;

//Filters
bool applyWindow = true;
bool applyLowPass = false;
float filterFrequency = 1.0; // in hz
float DIR_WINDOW_SIZE = 5.0;
float MAG_WINDOW_SIZE = 5.0;
bool wrapAround;
float previousDirectionHoVR;
float previousMagnitudeHoVR;
FilterOnePole lowpassFilterDirection( LOWPASS, filterFrequency ); 
FilterOnePole lowpassFilterMagnitude( LOWPASS, filterFrequency ); 

// This function determines if the change in direction was positive or negative 
bool doesItWrap(float previous, float current)
{
  float noWrap;
  float wrap;
  noWrap = abs(current - previous);
  if (previous < 180)
  {
    previous = previous + 360;
  }
  else
  {
    previous = previous - 360;
  }
  wrap = abs(current - previous);
  if (wrap > noWrap)
  {
    return false;
  }
  return true;
}

// This function applies the window filter function (other option is lowpass filter)
// This removes the high frequency noise
// This required the wrap around function when considering positive or negative change in direction
float applyWindowDirection(float previousDirectionHoVRW, float directionHoVRW)
{
  if ((directionHoVRW < (previousDirectionHoVRW - DIR_WINDOW_SIZE) && wrapAround == false) || (directionHoVRW > (previousDirectionHoVRW + DIR_WINDOW_SIZE) && wrapAround == true))
    {
    directionHoVRW = previousDirectionHoVRW - DIR_WINDOW_SIZE;
  } 
  else if ((directionHoVRW > (previousDirectionHoVRW + DIR_WINDOW_SIZE) && wrapAround == false) || (directionHoVRW < (previousDirectionHoVRW - DIR_WINDOW_SIZE) && wrapAround == true))
  {
    directionHoVRW = previousDirectionHoVRW + DIR_WINDOW_SIZE;
  }
  //Serial.println(int(directionHoVRW));
  //> or >=
  if (directionHoVRW > 360)
  {
    directionHoVRW = directionHoVRW - 360.0;
  } 
  else if (directionHoVRW < 0)
  {
    directionHoVRW = directionHoVRW + 360.0;
  }

  return directionHoVRW;
}

float applyWindowMagnitude(float previousMagnitudeHoVR, float magnitudeHoVR1)
{
  if (magnitudeHoVR1 < previousMagnitudeHoVR - MAG_WINDOW_SIZE)
  {
    magnitudeHoVR1 = previousMagnitudeHoVR - MAG_WINDOW_SIZE;
  }
  else if (magnitudeHoVR1 > previousMagnitudeHoVR + MAG_WINDOW_SIZE)
  {
    magnitudeHoVR1 = previousMagnitudeHoVR + MAG_WINDOW_SIZE;
  }

  return magnitudeHoVR1;
}

//// The second string value calculation for motor PWM////
int pwmValue;
int secondStringValue(int myStr2, int minPWM, int incrPWM){
  if (myStr2 == 0){
    pwmValue = 0;
  } else {
    pwmValue = minPWM + myStr2*incrPWM;
  }
  return pwmValue;
}


void setup()
{
  Wire.begin();
  Serial.begin(38400);
  // IMU Setup
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  
  if (c == 0x71)
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    
    // Calibrate gyro and accelerometers, load biases in bias registers
    // this requires the IMU to be still
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  //Calculating the accelerometer bias by measuring over 200 loops and averaging
  int numLoopsActual = 0;
  for (int i = 0; i < numLoops; i++){
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values accel
      myIMU.getAres();
      axBias += (float)myIMU.accelCount[0]*myIMU.aRes; 
      ayBias += (float)myIMU.accelCount[1]*myIMU.aRes; 
      azBias += (float)myIMU.accelCount[2]*myIMU.aRes;
      numLoopsActual = numLoopsActual + 1; // count the actual loops that obtain data
    }
  }
  axBias = axBias/numLoopsActual;
  ayBias = ayBias/numLoopsActual;
  azBias = azBias/numLoopsActual;

  // Set the PWM motor pins to output 
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);

  //char receivedChars[numChars];

} // End Setup

void loop() {
  //Get the accel and gyro data from the IMU
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values accel
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes - axBias;
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes - ayBias;
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes - (azBias + 1.0); // fixed a bug +1 instead of -1, depends on orientation 
    
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values gyro
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    // Convert the accelerometer values into pitch and roll angles
    accel_pitch = -atan2(myIMU.ax, sqrt(sq(myIMU.ay) + sq(myIMU.az)))*180/PI;
    accel_roll = atan2(myIMU.ay, sqrt(sq(myIMU.ax) + sq(myIMU.az)))*180/PI;
    
    //integrate the gyro DPS into pitch and roll anlges, then combine the accelerometer and gyro values using fusion formula
    dt = millis() - prevTime;
    prevTime = millis();
    if (firstRun) {
      combined_pitch = accel_pitch;
      combined_roll = accel_roll;
      gyro_pitch = accel_pitch;
      gyro_roll = accel_roll;
      firstRun = false;
    } else {
      // sensor fusion formula
      combined_pitch = hpf *( combined_pitch + myIMU.gy * (dt / 1000.0)) + lpf *(accel_pitch); 
      combined_roll = hpf *( combined_roll + myIMU.gx * (dt / 1000.0)) + lpf *(accel_roll);
  
      gyro_pitch = gyro_pitch + myIMU.gy * (dt / 1000.0);
      gyro_roll = gyro_roll + myIMU.gx * (dt / 1000.0);
    }
  
    
    //convert combined pitch and roll back to vectors
    xAxis = sin(combined_pitch * PI/180.0);
    yAxis = sin(combined_roll * PI/180.0);

    //calculate direction and magnitude from the vectors
    directionHoVR = -atan2(yAxis,xAxis)*180/PI + 180; // change the end value depending on the orientation of the IMU
    magnitudeHoVR1 = (atan2(sqrt(sq(yAxis) + sq(xAxis)), abs(myIMU.az))*180/PI);
    
    magnitudeHoVR1 = 5.5 * (pow(magnitudeHoVR1,4))/(pow(magnitudeHoVR1,4)+pow(2,4)); // returning to zero?
    // scale the magnitude and limit its maximum value
    if (magnitudeHoVR1 > 5.0){
      magnitudeHoVR1 = 100.0;
    } else {
      magnitudeHoVR1 = magnitudeHoVR1 * 20.0;
    }
    // The filtering of the signal. Signal window filter
    if (firstRun)
    {
      previousDirectionHoVR = directionHoVR;
      previousMagnitudeHoVR = magnitudeHoVR1;
      firstRun = false; //Potential Error, used twice and set to false above!!!
    }
  
    wrapAround = doesItWrap(previousDirectionHoVR, directionHoVR);

    //Choice between Window filter and LowPass filter
    if (applyWindow == true)
    {
      directionHoVR = applyWindowDirection(previousDirectionHoVR, directionHoVR);
      magnitudeHoVR1 = applyWindowMagnitude(previousMagnitudeHoVR, magnitudeHoVR1);
    }

    if (applyLowPass == true)
    {
      directionHoVR = lowpassFilterDirection.input( directionHoVR );
      magnitudeHoVR1 = lowpassFilterMagnitude.input( magnitudeHoVR1 );
    }
    previousDirectionHoVR = directionHoVR;
    previousMagnitudeHoVR = magnitudeHoVR1;
    
  } // if(myIMU.readByte... 
  // Write to the serial port after every 10ms
  TestTime = millis() - TestTimeUpdate;
  if (TestTime > 10)
  {
    //directionHoVR = (int(directionHoVR) + 90)%360;
    Serial.print(int(directionHoVR));
    Serial.print(",");
    Serial.println(int(magnitudeHoVR1));
    TestTimeUpdate = millis();
  }

  // function to parse serial read data
  recvWithStartEndMarkers();
  // function to handle new data and change motor case
  showNewData();
  // function to pulse motor for specified duration
  motorTimed();

} // End Loop

// function to parse serial read data
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = 'S';
    char endMarker = '\r';
    char rc;
 
 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// function to handle new data and change motor case
void showNewData() {
    if (newData == true) {
        //convert the string to integers for the different cases
        myStr = (String(receivedChars[0]) + String(receivedChars[1])).toInt();
        myStr1 = String(receivedChars[0]).toInt(); 
        myStr2 = String(receivedChars[1]).toInt();
        // check if the state has changed and switch to new case
        if (myStr != myStrLast){
          switch(myStr1){
            case 1:
              secondStringValue(myStr2, 100, 15);
              analogWrite(motor1, pwmValue);
              digitalWrite(motor2, LOW);
              break;
            case 2:
              analogWrite(motor1, LOW);
              digitalWrite(motor2, LOW);
              break;
            case 3:
              analogWrite(motor1, 75);
              digitalWrite(motor2, LOW);
              break;
            // the motor pulse case
            case 4:
              motorRun = true;
              time_start2 = millis();
              break;
            //if none of the above set motors to off
            default:
              digitalWrite(motor1, LOW);
              digitalWrite(motor2, LOW);
              break;
          }
        } 
        time_start = millis();
        newData = false;
        myStrLast = myStr;
    }
    else {
      // timeout if no new data is received within 1000ms.
      // So motors don't keep running
      int time_now = millis();
      int time_diff = time_now - time_start;
      if (time_diff > 1000) {
        digitalWrite(motor1, LOW);
        // ? digitalWrite(motor2, LOW);
        myStrLast = 0;
        time_start = millis();
      }
    }
}

// function to pulse motor for specified duration
void motorTimed() {
  if (motorRun == true) {
    secondStringValue(myStr2, 75, 15);
    analogWrite(motor2, pwmValue);
  }
  else {
    digitalWrite(motor2, LOW);
  }
  // when pulse time completes set motorRun false for LOW
  int time_now2 = millis();
  int time_diff2 = time_now2 - time_start2;
  if (time_diff2 > 300) {
    motorRun = false;
    time_start2 = millis();
  }
}
