package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private CRServo intakeServo;
    private Servo intakeLift;

    private int intakeDownPosition;
    private int intakeUpPosition;
    public void initIntake() {
        intakeServo.setPower(0);
    }

    public Intake(OpMode OpMode) {

    }

    public void liftIntake() {
        if (intakeLift.getPosition() <= 0.5)
            intakeLift.setPosition(intakeUpPosition);
        else {
            intakeLift.setPosition(intakeDownPosition);
    }

}


    public void collect(double power) {
        intakeServo.setPower(power);
    }

}
