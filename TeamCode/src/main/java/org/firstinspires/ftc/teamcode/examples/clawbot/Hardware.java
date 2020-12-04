package org.firstinspires.ftc.teamcode.examples.clawbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.servo.Servo;

//class for hardware
public class Hardware {
    //drive motors
    public Motor<DcMotor> leftMotor, rightMotor;

    //claw arm
    public EncodedMotor<DcMotor> armMotor;

    //claw
    public Servo clawServo;

    //create all devices with correct names in hardware map
    public Hardware(){
        leftMotor = new Motor<>("left");
        rightMotor = new Motor<>("right").setInverted(true);

        armMotor = new EncodedMotor<>("arm");

        clawServo = new Servo("claw");
    }
}
