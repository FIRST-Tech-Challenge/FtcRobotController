package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private CRServo intakeServo;
    private Servo intakeLift;


    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        intakeLift = hardwareMap.get(Servo.class, "IntakeLift");
    }

    public void liftIntake(double position) {
        intakeLift.setPosition(position);
    }

    public void take(double power) {
        intakeServo.setPower(power);
    }

}
