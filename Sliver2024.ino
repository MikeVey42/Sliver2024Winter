#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

// KEYBINDS
Key startAutoKey = Key.Digit0;
Key shootKey = Key.I;
Key intakeKey = Key.J;

// Drivetrain Motors
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);

// Flywheels: for launching notes
NoU_Motor leftFlywheel(5);
NoU_Motor rightFlywheel(6);

// Intake: for picking up notes
NoU_Motor intakeMotor(7);

// Aiming: these are for aiming the shooter left/right and up/down
NoU_Servo xAlignServo(1);
float minimumAngle = 0;
float maximumAngle = 180;
float startAngle = 180;
NoU_Servo yAlignServo(2);

// Sensor Servo: this is for changing the angle of the distance sensor to be horizontal with the ground 
NoU_Servo distanceSensorServo(3);

// The Drivetrain object handles the arcade drive math for us
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE BEFORE PAIRING THEIR ROBOT TO ANY LAPTOP
    NoU3.begin();
    PestoLink.begin("NoU3_Bluetooth");
    Serial.begin(115200);

    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    rearLeftMotor.setInverted(false);
    rearRightMotor.setInverted(true);
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

    // Here we decide what the servo angle will be based on if button 0 is pressed
    int servoAngle = 0;

    if (PestoLink.buttonHeld(0)) {
        servoAngle = 70;
    }
    else {
        servoAngle = 110;
    }

    // Here we set the drivetrain motor speeds and servo angle based on what we found in the above code
    servo.write(servoAngle);

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

float getYaw() {
  // TODO: check if this is the right axis
  return NoU3.gyroscope_x;
}

