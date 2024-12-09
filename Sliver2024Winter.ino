#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <Keys.h>

// KEYBINDS
Key startAutoKey = Key::Digit0;
Key aimKey = Key::I;
Key fireKey = Key::U;
Key intakeKey = Key::J;
Key revIntakeKey = Key::H;
Key ampKey = Key::O;

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


// Aiming: these are for aiming the shooter left/right and up/down
NoU_Servo xAlignServo(1);
    float xStartAngle = -10;
NoU_Servo yAlignServo(3);
    float yStowAngle = 160;
    float yMeasureAngle = 160;

// Sensor Servo: this is for changing the angle of the distance sensor to be horizontal with the ground 
// NoU_Servo distanceSensorServo(3);
//     float sensorStowAngle = 90;
//     float sensorMeasureAngle = 90;
//     float sensorAutoAngle = 90;

// The Drivetrain object handles the arcade drive math for us
NoU_Drivetrain drivetrain(&leftMotor, &rightMotor);

// GYRO
float yaw_gyro_deg = 0;
float yaw_mag_deg = 0;
float wrapped_yaw_mag_deg = 0;
float yaw = 0;

const float alpha = 0.98; // Complementary filter weighting
unsigned long lastTime = 0;

float gyro_z_offset_degrees = -0.14;

boolean firing = false;
unsigned long fireTime = 0;

boolean amp = false;
unsigned long ampTime = 0;

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE BEFORE PAIRING THEIR ROBOT TO ANY LAPTOP
    NoU3.begin();
    PestoLink.begin("Sliver24v2");
    Serial.begin(115200);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
}

void loop() {

    // This measures your batteries voltage and sends it to PestoLink
    // You could use this value for a lot of cool things, for example make LEDs flash when your batteries are low?
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
    if (PestoLink.update()) {
        
        drive();

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    updateGyro();

    turretControl();
    // No need to mess with this code
    PestoLink.update();
    NoU3.updateServiceLight();
    NoU3.updateIMUs();
}

void drive() {
  float throttle = PestoLink.getAxis(1);
  float rotation = 0.9 * -PestoLink.getAxis(0);
  
  drivetrain.arcadeDrive(throttle, rotation);
}

void turretControl() {
  if (PestoLink.keyHeld(fireKey)) {
    prepareToShoot();
    fire();
  }else {
    firing = false;
    indexerMotor.set(0);
    if (PestoLink.keyHeld(aimKey)) {
      prepareToMeasure();
      leftFlywheel.set(0);
      rightFlywheel.set(0);
    }
    else if (PestoLink.keyHeld(intakeKey)) {
      intake();
    } else if (PestoLink.keyHeld(revIntakeKey)) {
      revIntake();
    }
    else {
      if (PestoLink.keyHeld(ampKey)) {
        doAmp();
      }else {
        stow();
        amp = false;
        leftFlywheel.set(0);
        rightFlywheel.set(0);
      }
    }
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
float xAlignWithSpeaker() {
    float currentYaw = getYaw();
    float targetAngle = (xStartAngle - currentYaw);
    // Print gyro values
    char result[8];
    dtostrf(currentYaw, 6, 2, result);
    //PestoLink.print(result);
    if (targetAngle > -10 && targetAngle < 190) {
        xAlignServo.write(targetAngle);
        return true;
    }else {
        xAlignServo.write(0);
        return false;
    }
}

// Uses a distance sensor to get the distance from the wall
float getDistance() {
    // unimplemented
    return 0;
}

// Uses a regression to find the ideal vertical (y) angle for the shooter given a distance
float getTargetShooterAngle() {
    // unimplemented
    float distance = getDistance();
    return 190;
}

void stow() {
    yAlignServo.write(yStowAngle);
    xAlignServo.write(180);
    //distanceSensorServo.write(sensorStowAngle);
    intakeServo.write(0);
    intakeMotor.set(0);
    indexerMotor.set(0);
}

void intake() {
  xAlignServo.write(180);
  yAlignServo.write(120);
  intakeServo.write(120);
  intakeMotor.set(1);
  leftFlywheel.set(-1);
  rightFlywheel.set(1);
  indexerMotor.set(-1);
}

void revIntake() {
  intakeServo.write(60);
  intakeMotor.set(-1);
}

void prepareToMeasure() {
  xAlignWithSpeaker();
  yAlignServo.write(yMeasureAngle);
  // distanceSensorServo.write(sensorMeasureAngle);
  indexerMotor.set(0);
}

void prepareToShoot() {
    xAlignWithSpeaker();
    float targetAngle = getTargetShooterAngle();
    yAlignServo.write(targetAngle);
    leftFlywheel.set(1);
    rightFlywheel.set(-0.5);
}

void fire() {
  if (firing) {
    if (millis() - fireTime > 1500) {
      indexerMotor.set(1);
    }
  }else {
    firing = true;
    fireTime = millis();
    indexerMotor.set(0);
  }
}

void doAmp() {
  xAlignServo.write(xStartAngle);
  yAlignServo.write(175);
  leftFlywheel.set(0.5);
  rightFlywheel.set(-0.5);
  if (amp) {
    if (millis() - ampTime > 1500) {
      indexerMotor.set(1);
    }
  }else {
    amp = true;
    ampTime = millis();
    indexerMotor.set(0);
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
        yaw_mag_deg = -1 * degrees(atan2(NoU3.magnetometer_y, NoU3.magnetometer_x));

        // Wrap Magnetometer yaw
        //wrapped_yaw_mag_deg = wrapYaw(yaw_mag_deg);

        // Apply complementary filter with drift compensation
        yaw = wrapYaw(yaw_gyro_deg);

        // // Print results
        // Serial.print("yaw_gyro_deg: ");
        // Serial.print(yaw_gyro_deg);
        // Serial.print(" wrapped_yaw_mag_deg: ");
        // Serial.print(wrapped_yaw_mag_deg);
        // Serial.print(" Yaw: ");
        // Serial.print(yaw);
        
        // Serial.println(" ");

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