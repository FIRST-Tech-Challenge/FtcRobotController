package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    Servo intakeServo; // Servo to control in the intake(for blocks)
    Servo pivotServo;  // Servo to control the pivot(for specimens)

    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");  // Initialize the pivot servo
    }

    // Method to control the intake motor
    public void setIntakePower(double power) {
        intakeServo.setPosition(power);
    }

    // Method to set the pivot servo position
    public void setPivotPosition(double position) {
        pivotServo.setPosition(position);
    }
}
