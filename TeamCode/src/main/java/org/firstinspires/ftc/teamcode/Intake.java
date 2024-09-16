package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotor intakeMotor;
    Servo pivotServo;  // Servo to control the pivot

    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");  // Initialize the pivot servo
    }

    // Method to control the intake motor
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    // Method to set the pivot servo position
    public void setPivotPosition(double position) {
        pivotServo.setPosition(position);
    }
}
