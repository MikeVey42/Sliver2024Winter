#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <Keys.h>

// KEYBINDS
Key startAutoKey = Key::Digit0;
Key shootKey = Key::I;
Key intakeKey = Key::J;

// Drivetrain Motors
NoU_Motor leftMotor(1);
NoU_Motor rightMotor(8);

// Flywheels: for launching notes
// NoU_Motor leftFlywheel(5);
// NoU_Motor rightFlywheel(6);

// Intake: for picking up notes
// NoU_Motor intakeMotor(7);

// Aiming: these are for aiming the shooter left/right and up/down
NoU_Servo xAlignServo(1);
    float xStartAngle = 180;
// NoU_Servo yAlignServo(2);
//     float yStowAngle = 60;

// Sensor Servo: this is for changing the angle of the distance sensor to be horizontal with the ground 
// NoU_Servo distanceSensorServo(3);
//     float sensorStowAngle = 90;
//     float sensorMeasureAngle = 90;
//     float sensorAutoAngle = 90;

// The Drivetrain object handles the arcade drive math for us
NoU_Drivetrain drivetrain(&leftMotor, &rightMotor);

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
    PestoLink.printBatteryVoltage(getYaw());

    // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
    if (PestoLink.update()) {
        
        drive();

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    // Here we decide what the servo angle will be based on if button 0 is pressed
    int servoAngle = 0;

    if (PestoLink.keyHeld(shootKey)) {
        servoAngle = 0;
    }
    else {
        servoAngle = 180;
    }

    // Here we set the drivetrain motor speeds and servo angle based on what we found in the above code
    xAlignServo.write(servoAngle);

    // No need to mess with this code
    PestoLink.update();
    NoU3.updateServiceLight();
    NoU3.updateIMUs();
}

void drive() {
  float throttle = -PestoLink.getAxis(1);
  float rotation = PestoLink.getAxis(0);
  
  drivetrain.arcadeDrive(throttle, rotation);
}

// Returns the yaw in degrees, CCW positive, from -180 to 180
float getYaw() {
  // TODO: check if this is the right axis
  return -NoU3.gyroscope_z;
}

// Moves the shooter to aim directly towards the alliance wall.
// If the robot is in front of the speaker, this will aim it towards the speaker
float xAlignWithSpeaker() {
    float currentYaw = getYaw();
    float targetAngle = (xStartAngle - currentYaw);
    if (targetAngle > 0 && targetAngle < 180) {
        xAlignServo.write(targetAngle);
        return true;
    }else {
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
    return 0;
}

// void stow() {
//     yAlignServo.write(yStowAngle);
//     xAlignServo.write(0)
//     distanceSensorServo.write(sensorStowAngle);
// }

// void prepareToMeasure() {
//     yStowAngle.write(yStowAngle);
//     distanceSensorServo.write(sensorMeasureAngle);
// }

// void prepareToShoot() {
//     float targetAngle = getTargetShooterAngle();
//     yAlignServo.write(yStowAngle);
// }