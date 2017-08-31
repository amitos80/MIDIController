
#include "Controller.h"
#include "quaternionFilters.h"
#include "MPU9250.h"
//****************************************************************************************
Mux::Mux(byte outpin_, byte numPins_, bool analog_)
{
  outpin = outpin_;
  //enablepin = enablepin_;
  numPins = numPins_;
  analog = analog_;
  if (analog == false) pinMode(outpin, INPUT_PULLUP);
  //pinMode(enablepin, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  if (numPins > 8) pinMode(5, OUTPUT);
}
//****************************************************************************************
//Button (Pin Number, Command, Note Number, Channel, Debounce Time)
Button::Button(byte pin, byte command, byte value, byte channel, byte debounce)
{
  _pin = pin;
  pinMode(_pin, INPUT_PULLUP);
  _value = value;
  _command = command;
  _debounce = debounce;
  _time = 0;
  _busy = false;
  _status = 0b00000010;
  _last = 1;
  Bcommand = command;
  Bvalue = value;
  Bchannel = channel;
  Btoggle = 0;
}

Button::Button(Mux mux, byte muxpin, byte command, byte value, byte channel, byte debounce)
{
  _pin = mux.outpin;
  _numMuxPins = mux.numPins;
  _muxpin = muxpin;
  _value = value;
  _command = command;
  _debounce = debounce;
  _time = 0;
  _busy = false;
  _status = 0b00000010;
  _last = 1;
  Bcommand = command;
  Bvalue = value;
  Bchannel = channel;
  Btoggle = 0;
}

void Button::muxUpdate()
{
  byte temp = _muxpin;
  temp = temp << 2;
  if (_numMuxPins > 8) PORTD = PORTD & B11000011;
  else PORTD = PORTD & B11100011;
  PORTD = PORTD | temp;
}

byte Button::getValue()
{
  // If BUSY bit not set - read button
  if (bitRead(_status, 0) == false) { // If busy false
    if (digitalRead(_pin) == _last) return 2; // If same as last state - exit
  }

  // If NEW Bit set - Key just pressed, record time
  if (bitRead(_status, 1) == true) { // If new is true
    bitSet(_status, 0); // Set busy TRUE
    bitClear(_status, 1); // Set New FALSE
    _time = millis();
    return 255;
  }

  // Check if debounce time has passed - If no, exit
  if (millis() - _time < _debounce) return 255;

  // Debounce time has passed. Read pin to see if still set the same
  // If it has changed back - assume false alarm
  if (digitalRead(_pin) == _last) {
    bitClear(_status, 0); // Set busy false
    bitSet(_status, 1); // Set new true
    return 255;
  }

  // If this point is reached, event is valid. return event type
  else {
    bitClear(_status, 0); // Set busy false
    bitSet(_status, 1); // Set new true
    _last = ((~_last) & 0b00000001); // invert _last
    return _last;
  }
}

void Button::newValue(byte command, byte value, byte channel)
{
  Bvalue = value;
  Bcommand = command;
  Bchannel = channel;
}

//********************************************************************
Pot::Pot(byte pin, byte command, byte control, byte channel)
{
  _pin = pin;
  _control = control;
  _value = analogRead(_pin);
  _value = _value >> 3;
  _oldValue = _value << 3;
  _value = _value << 3;
  Pcommand = command;
  Pcontrol = control;
  Pchannel = channel;
}

void Pot::muxUpdate()
{
  byte temp = _muxpin;
  temp = temp << 2;
  if (_numMuxPins > 8) PORTD = PORTD & B11000011;
  else PORTD = PORTD & B11100011;
  //PORTD = PORTD & B11000011;
  PORTD = PORTD | temp;
}

Pot::Pot(Mux mux, byte muxpin, byte command, byte control, byte channel)
{
  _pin = mux.outpin;
  _numMuxPins = mux.numPins;
  _muxpin = muxpin;
  _control = control;
  muxUpdate();
  _value = analogRead(_pin);
  _value = _value >> 3;
  _oldValue = _value << 3;
  _value = _value << 3;
  Pcommand = command;
  Pcontrol = control;
  Pchannel = channel;
}

byte Pot::getValue()
{
  _value = analogRead(_pin);
  int tmp = (_oldValue - _value);
  if (tmp >= 8 || tmp <= -8) {
    _oldValue = _value >> 3;
    _oldValue = _oldValue << 3;
    return _value >> 3;
  }
  return 255;
}

void Pot::newValue(byte command, byte value, byte channel) {
  Pcommand = command;
  Pcontrol = value;
  Pchannel = channel;
}

Accl::Accl(int c){}

void Accl::init() {
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  //Serial.begin(38400);
  Serial.begin(9600);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    // Communication failed, stop here
    Serial.println(F("1 Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void Accl::updateValues() {
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
 }

