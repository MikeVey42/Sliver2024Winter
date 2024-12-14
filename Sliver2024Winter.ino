#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <Keys.h>
#include <math.h>

// KEYBINDS
Key startAutoKey = Key::Digit0;
Key aimKey = Key::I;
Key fireKey = Key::U;
Key intakeKey = Key::J;
Key revIntakeKey = Key::H;
Key ampKey = Key::O;

Key moveOnlyAuto = Key::Digit1;
Key centerCloseAuto = Key::Digit2;
Key centerFarAuto = Key::Digit3;
Key sourceCloseAuto = Key::Digit4;
Key sourceFarAuto = Key::Digit5;
Key armRedAuto = Key::Digit8;
Key armBlueAuto = Key::Digit9;
Key terminateAuto = Key::Digit0;

Key increaseAngleKey = Key::T;
Key decreaseAngleKey = Key::G;

// Drivetrain Motors
NoU_Motor leftMotor(1);
NoU_Motor rightMotor(8);

// Flywheels: for launching notes
NoU_Motor leftFlywheel(5);
NoU_Motor rightFlywheel(7);

NoU_Motor indexerMotor(6);

// Intake: for picking up notes
NoU_Servo intakeServo(2);
    float intakeStartAngle = 0;
NoU_Motor intakeMotor(3);

// Autos
int autoSequence = 0;
int autoStartTime = 0;
int autoTimer = 0;
bool autoStarted = false;
bool isRed = true;

// Aiming: these are for aiming the shooter left/right and up/down
NoU_Servo xAlignServo(1);
    float xStartAngle = -10;
NoU_Servo yAlignServo(3);
    float yStowAngle = 130;
    float yMeasureAngle = 130;

// Sensor Servo: this is for changing the angle of the distance sensor to be horizontal with the ground 
NoU_Servo distanceSensorServo(4);
    float sensorStowAngle = 145;
    float sensorMeasureAngle = 90;
    float sensorAutoAngle = 100;

  int trigPin = 9;
  int echoPin = 8;

// The Drivetrain object handles the arcade drive math for us
NoU_Drivetrain drivetrain(&leftMotor, &rightMotor);
float throttle = 0;
float rotation = 0;

// Turning variables
boolean turnStarted;
int startDegrees = 0;

// GYRO
float yaw_gyro_deg = 0;
float yaw_mag_deg = 0;
float wrapped_yaw_mag_deg = 0;
float yaw = 0;

const float alpha = 0.98; // Complementary filter weighting
unsigned long lastTime = 0;

float gyro_z_offset_degrees = -0.14;

enum State{
  stow, 
  intaking,
  outtaking,
  intermediate,
  aiming,
  spinningUp,
  firing,
  prepareAmp,
  scoreAmp,
  autonomous
};

State state = stow;
State targetState = stow;
unsigned long lastStateChange = 0;
bool scoreInputLastLoop = false;

unsigned long lastStartDrive = 0;
bool driving = false;

float targetYAngle = 180;
bool lastChangeAngle;

unsigned long startEcho = 0;
bool echoing = false;

void setup() {
  NoU3.begin();
  PestoLink.begin("Sliver24v2");
  Serial.begin(115200);

  leftMotor.setInverted(true);
  rightMotor.setInverted(false);

  // Distance sensor 
  pinMode(trigPin, OUTPUT); // This is the trigger pin, it tells the sensor to fire
  pinMode(echoPin, INPUT_PULLUP); // This is the echo pin, it reads the distance

}

void loop() {
  throttle = 0;
  rotation = 0;

  autoTimer = millis() - autoStartTime;

  // This measures your batteries voltage and sends it to PestoLink
  // You could use this value for a lot of cool things, for example make LEDs flash when your batteries are low?
  float batteryVoltage = NoU3.getBatteryVoltage();
  //PestoLink.printBatteryVoltage(batteryVoltage);

  if (PestoLink.keyHeld(decreaseAngleKey) && !lastChangeAngle) {
    targetYAngle--;
    print(targetYAngle);
  }else if (PestoLink.keyHeld(increaseAngleKey) && !lastChangeAngle) {
    targetYAngle++;
    print(targetYAngle);
  }
  lastChangeAngle = PestoLink.keyHeld(decreaseAngleKey) || PestoLink.keyHeld(increaseAngleKey);

  // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
  if (PestoLink.update()) {
    
    drive();

    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.setServiceLight(LIGHT_DISABLED);
  }

  updateGyro();

  updateState();
  performState();
    
  // Wait until the end of the loop to update components to reduce glitches
  drivetrain.curvatureDrive(throttle, rotation);

  // No need to mess with this code
  PestoLink.update();
  NoU3.updateServiceLight();
  NoU3.updateIMUs();
}

void drive() {
  if ((state == intaking || state == outtaking) && stateTime() < 0.25 && throttle == 0 && rotation == 0) {
    throttle = 0;
    rotation = 0;
    driving = false;
    return;
  }

  bool W = PestoLink.keyHeld(Key::W);
  bool S = PestoLink.keyHeld(Key::S);
  bool A = PestoLink.keyHeld(Key::A);
  bool D = PestoLink.keyHeld(Key::D);

  if (!driving && (W || S || A || D)) {
    lastStartDrive = millis();
  }

  if (W) {
    throttle = -1;
    driving = true;
  }else if (S) {
    throttle = 1;
    driving = true;
  }else{
    throttle = throttle * 0.5;
    driving = false;
  }

  if (A) {
    rotation = 0.9;
    driving = true;
  }else if (D) {
    rotation = -0.9;
    driving = true;
  }else{
    rotation = 0;
    driving = false;
  }
}

void autoDrive(float t, float r) {
  throttle = t;
  rotation = r;
}

void changeStateTo(State s) {
  state = s;
  lastStateChange = millis();
}

float stateTime() {
  return millis() - lastStateChange;
}

void updateState() {
  // If in intermediate, go to next
  if (state == intermediate) {
    if (stateTime() > 500) {
      changeStateTo(targetState);
    }
  }else if (state == firing) {
    if (stateTime() > 1000) {
      targetState = stow;
      changeStateTo(intermediate);
    }
  }else if (state == spinningUp) {
    if (!PestoLink.keyHeld(fireKey) && !autoStarted) {
      targetState = stow;
      changeStateTo(intermediate);
    }
    else if (stateTime() > 1000) {
      changeStateTo(firing);
    }
  }else if (state == scoreAmp) {
    if (stateTime() > 1000) {
      targetState = stow;
      changeStateTo(intermediate);
    }
  }else if (state == prepareAmp) {
    if (!PestoLink.keyHeld(ampKey)) {
      targetState = stow;
      changeStateTo(intermediate);
    }
    else if (stateTime() > 1000) {
      changeStateTo(scoreAmp);
    }
  }else if (state == autonomous) {
    if(autoSequence == 0) {
      targetState = stow;
      changeStateTo(intermediate);
      updateAutos();
    }
  }else {
    if (state == aiming) {
      if (PestoLink.keyHeld(fireKey) && !scoreInputLastLoop) {
        measure();
        changeStateTo(spinningUp);
      }
    } else if (PestoLink.keyHeld(intakeKey)) {
      if (millis() - lastStartDrive > 250) {
        changeStateTo(intaking);
      }
    } else if (PestoLink.keyHeld(revIntakeKey)) {
      if (millis() - lastStartDrive > 250) {
        changeStateTo(outtaking);
      }
    } else if (PestoLink.keyHeld(aimKey)) {
      changeStateTo(aiming);
    } else if (PestoLink.keyHeld(ampKey) && !scoreInputLastLoop) {
      changeStateTo(prepareAmp);
    } else{
      changeStateTo(stow);
    }

    updateAutos();
  }
  scoreInputLastLoop = PestoLink.keyHeld(ampKey) || PestoLink.keyHeld(fireKey);
}

void updateAutos() {
    if(PestoLink.keyHeld(moveOnlyAuto)) {
      autoSequence = 1;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(centerCloseAuto)) {
      autoSequence = 2;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(centerFarAuto)) {
      autoSequence = 3;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(sourceCloseAuto)) {
      autoSequence = 4;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(sourceFarAuto)) {
      autoSequence = 5;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(armRedAuto)) {
      autoSequence = 8;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(armBlueAuto)) {
      autoSequence = 9;
      changeStateTo(autonomous);
    } else if(PestoLink.keyHeld(terminateAuto)) {
      autoSequence = 0;
    }
}

void performState() {
  switch (state)
  {
    case stow:
      doStow();
      break;
    
    case intaking:
      doIntaking();
      break;

    case outtaking:
      doOuttaking();
      break;

    case intermediate:
      doIntermediate();
      break;

    case aiming:
      doAiming();
      break;

    case spinningUp:
      doSpinUp();
      break;

    case firing:
      doFire();
      break;

    case prepareAmp:
      doPrepareAmp();
      break;

    case scoreAmp:
      doScoreAmp();
      break;
    
    case autonomous:
      print(autoSequence);
      switch(autoSequence) {
        case 1:
          moveAuto();
          break;
        case 2:
          centerAuto();
          break;
        case 3:
          //centerAltAuto();
          break;
        case 4:
          sourceAuto();
          break;
        case 5:
          //sourceAltAuto();
          break;
        case 8:
          doArmAuto(true);
          break;
        case 9:
          doArmAuto(false);
          break;
        default:
          autoStarted = false;
          return;
      }
      break;

    default:
      break;
  }
}

void doArmAuto(bool redAlliance) {
  isRed = redAlliance;
  if(stateTime() < 1000) {
    doAiming(true, 0, yStowAngle);
  } else {
    doAiming(true, 0, 170); // Prep for rear subwoofer shot
  }
}

void moveAuto() {
  if(autoStarted == false) {
    autoStarted = true;
    autoStartTime = millis();
  } else if(autoTimer < 1000) {
    throttle = -1;
    return;
  } else {
    autoSequence = 0;
    autoStarted = false;
  }
}

void centerAuto() {
  if(autoStarted == false) {
    autoStarted = true;
    autoStartTime = millis();
  } else if(autoTimer < 1000) {
    doSpinUp(true);
  } else if(autoTimer < 2000) {
    doFire(true);
  } else if(autoTimer < 2200) {
    doStow();
  } else if(autoTimer < 2500) {
    doIntaking();
  } else if(autoTimer < 3500) {
    throttle = -1;
    doIntaking();
  } else if(autoTimer < 4500) {
    throttle = 1;
    doIntaking();
  } else if(autoTimer < 5000) {
    doStow();
  } else if(autoTimer < 6000) {
    doAiming(true, 0, 190); // Prep for rear subwoofer shot
    doSpinUp(true);
  } else if(autoTimer < 7000) {
    doFire(true);
  } else if(autoTimer < 7200) {
    doStow();
  } else if(autoTimer < 7500) {
    doIntaking();
  } else if(autoTimer < 7550) {
    turnInit();
  } else if(autoTimer < 8500) {
    throttle = -1;
    turnBy(isRed, 45);
    doIntaking();
  } else if(autoTimer < 8550) {
    turnInit();
  } else if(autoTimer < 9500) {
    throttle = 1;
    turnBy(!isRed, 45);
    doIntaking();
  } else if(autoTimer < 10000) {
    doAiming(true, 0, 170); // Prep for rear subwoofer shot
    doSpinUp(true);
  } else if(autoTimer < 11000) {
    doFire(true);
  } else {
    autoSequence = 0;
    autoStarted = false;
  }
}

void sourceAuto() {
  if(autoStarted == false) {
    autoStarted = true;
    autoStartTime = millis();
  } else if(autoTimer < 1000) {
    doSpinUp(true);
  } else if(autoTimer < 2000) {
    doFire(true);
  } else if(autoTimer < 2200) {
    doStow();
  } else if(autoTimer < 2500) {
    doIntaking();
  } else if(autoTimer < 2550) {
    turnInit();
  } else if(autoTimer < 3500) {
    throttle = -1;
    turnBy(isRed, 45);
    doIntaking();
  } else if(autoTimer < 3550) {
    turnInit();
  } else if(autoTimer < 4500) {
    throttle = 1;
    turnBy(!isRed, 45);
    doIntaking();
  } else if(autoTimer < 5000) {
    doStow();
  } else if(autoTimer < 6000) {
    doAiming(true, 0, 190); // Prep for rear subwoofer shot
    doSpinUp(true);
  } else if(autoTimer < 7000) {
    doFire(true);
  } else if(autoTimer < 7200) {
    doStow();
  } else {
    autoSequence = 0;
    autoStarted = false;
  }
}

void turnInit() {
  turnStarted = false;
}

void turnBy(bool clockwise, int degrees) {
  if(!turnStarted) { 
    startDegrees = getYaw(); 
    turnStarted = true;
  }
  if(abs(startDegrees - getYaw()) < degrees - 5) {
    if(clockwise) { rotation = 1; }
    else { rotation = -1; }
  }
}

// Returns the yaw in degrees, CCW positive, from -180 to 180
float getYaw() {
  // TODO: check if this is the right axis
  return yaw;
  //return -NoU3.magnetometer_z;
}

// Moves the shooter to aim directly towards the alliance wall.
// If the robot is in front of the speaker, this will aim it towards the speaker
float getXAngle() {
  float currentYaw = getYaw();
  float targetYaw = currentYaw - getJoystickAngle();
  if (targetYaw > 180) {
    targetYaw -= 360;
  }else if (targetYaw < -180) {
    targetYaw += 360;
  }
  float targetAngle = (xStartAngle - targetYaw);
  if (targetAngle > -10 && targetAngle < 190) {
      return targetAngle;
  }else {
      return -10;
  }
}

float getJoystickAngle() {
  float x = PestoLink.getAxis(0);
  //print(x);
  float y = PestoLink.getAxis(1);

  float rotation = atan2(x, -y);
  rotation *= (180.0 / 3.1415926);
  if (rotation > 180) {
    rotation -= 360;
  }
  return rotation;
}

void doStow() {
  yAlignServo.write(yStowAngle);
  xAlignServo.write(180);
  distanceSensorServo.write(sensorStowAngle);

  runFlywheels(0);
  indexerMotor.set(0);

  intakeServo.write(0);
  intakeMotor.set(0);
}

void doIntaking() {
  yAlignServo.write(80);
  xAlignServo.write(185);
  distanceSensorServo.write(sensorStowAngle);

  runFlywheels(-1);
  indexerMotor.set(-1);

  intakeServo.write(120);
  intakeMotor.set(1);
}

void doOuttaking() {
  yAlignServo.write(yStowAngle - 20);
  xAlignServo.write(180);
  distanceSensorServo.write(sensorStowAngle);

  runFlywheels(1);
  indexerMotor.set(1);

  intakeServo.write(60);
  intakeMotor.set(-1);
}

void doIntermediate() {
  yAlignServo.write(yMeasureAngle);
  xAlignServo.write(180);
  distanceSensorServo.write(sensorStowAngle);

  runFlywheels(0);
  indexerMotor.set(0);

  intakeServo.write(0);
  intakeMotor.set(0);
}

void doAiming() {
  doAiming(false, 0, 0);
}

void doAiming(bool manual, int x, int y) {
  if(manual) {
    yAlignServo.write(y);
    xAlignServo.write(x);
  } else {
    yAlignServo.write(yMeasureAngle);
    xAlignServo.write(getXAngle());
  }
  distanceSensorServo.write(sensorMeasureAngle);

  runFlywheels(0);
  indexerMotor.set(0);

  intakeServo.write(0);
  intakeMotor.set(0);
}

float measureOnce() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration) * .0343 / 2;
  
  return distance;
}

void measure() {
  // // Measure multiple times so we can throw out outliers.
  // // This compensates for the sensor being highly consistent most of the time, but occaisonally being WAY off
  // int measurements[3];
  // int mean = 0;
  // for (int i = 0; i < 3; i++) {
  //   measurements[i] = measureOnce();
  //   mean = mean + measurements[i];
  // }
  // mean /= 3;
  // int distances[3];
  // for (int i = 0; i < 3; i++) {
  //   distances[i] = abs(mean - measurements[i]);
  // }

  // float finalDistance = 0;

  // // Throw out the worst value (whichever is furthest from the mean)
  // if (distances[0] > distances[1] && distances[0] > distances[2]) {
  //   finalDistance = (measurements[1] + measurements[2]) / 2;
  // }else if (distances[1] > distances[0] && distances[1] > distances[2]) {
  //   finalDistance = (measurements[0] + measurements[2]) / 2;
  // }else {
  //   finalDistance = (measurements[0] + measurements[1]) / 2;
  // }
  targetYAngle = getTargetShooterAngle(measureOnce());
}

void doSpinUp() {
  doSpinUp(false);
}

void doSpinUp(bool manual) {
  if(!manual) {
    yAlignServo.write(targetYAngle);
    xAlignServo.write(getXAngle());
  }
  distanceSensorServo.write(sensorMeasureAngle);

  leftFlywheel.set(1);
  rightFlywheel.set(-0.5);
  indexerMotor.set(0);

  intakeServo.write(0);
  intakeMotor.set(0);
}

void doFire() {
  doFire(false);
}

void doFire(bool manual) {
  if(!manual) {
    yAlignServo.write(targetYAngle);
    xAlignServo.write(getXAngle());
  }
  distanceSensorServo.write(sensorStowAngle);

  leftFlywheel.set(1);
  rightFlywheel.set(-0.5);
  indexerMotor.set(1);

  intakeServo.write(0);
  intakeMotor.set(0);
}

void doPrepareAmp() {
  yAlignServo.write(145);
  xAlignServo.write(xStartAngle);
  distanceSensorServo.write(sensorStowAngle);

  runFlywheels(0.5);
  indexerMotor.set(0);

  intakeServo.write(0);
  intakeMotor.set(0);
}

void doScoreAmp() {
  yAlignServo.write(145);
  xAlignServo.write(xStartAngle);
  distanceSensorServo.write(sensorStowAngle);

  runFlywheels(0.5);
  indexerMotor.set(1);

  intakeServo.write(0);
  intakeMotor.set(0);
}

// Runs both flywheels. -1 is intake, 1 is outtake/shoot.
void runFlywheels(float power) {
  leftFlywheel.set(power);
  rightFlywheel.set(-power);
}

// Uses a regression to find the ideal vertical (y) angle for the shooter given a distance
float getTargetShooterAngle(float distance) {
  print(distance);
  if(distance > 92.35) {
    return 150;
  }else {
    float a = 0.00576007 * distance * distance;
    float b = -1.06389 * distance;
    float c = 199.12046;
    return a + b + c;
  }
}

void updateGyro() {
  if (NoU3.updateIMUs()) {

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Gyroscope yaw update
    yaw_gyro_deg += (NoU3.gyroscope_z - gyro_z_offset_degrees) * dt;

    // Magnetometer yaw
    //yaw_mag_deg = -1 * degrees(atan2(NoU3.magnetometer_y, NoU3.magnetometer_x));

    // Wrap Magnetometer yaw
    //wrapped_yaw_mag_deg = wrapYaw(yaw_mag_deg);

    // Wrap yaw
    yaw = wrapYaw(yaw_gyro_deg);

  }
}

float wrapYaw(float currentYaw) {
  static bool isInitialized = false;
  static int rotations = 0;
  static float previousYaw = 0;

  if(isInitialized == false){
    previousYaw = currentYaw;
    isInitialized = true;
  }

    // Check for wrapping
    if (currentYaw - previousYaw > 180.0) {
        rotations--; // Wrapped from -180 to 180
    } else if (currentYaw - previousYaw < -180.0) {
        rotations++; // Wrapped from 180 to -180
    }

    // Wrap the yaw angle between -180 and 180
    float wrappedYaw = currentYaw + rotations * 360.0;

    previousYaw = currentYaw;

    return wrappedYaw;
}

void print (float value) {
  char result[8];
  dtostrf(value, 6, 2, result);
  PestoLink.print(result);
}