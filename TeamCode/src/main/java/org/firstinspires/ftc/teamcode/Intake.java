package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private CRServo intakeServo;
    private Servo intakeLift;

    private int intakeDownPosition = 0;
    private int intakeUpPosition = 1;
    public void initIntake() {
        intakeServo.setPower(0);
    }

    public Intake(OpMode OpMode) {

    }

    public void liftIntake() {
        if (intakeLift.getPosition() <= intakeUpPosition)
            intakeLift.setPosition(intakeUpPosition);
        else {
            intakeLift.setPosition(intakeDownPosition);
    }

}


    public void collect(double power) {
        intakeServo.setPower(power);
    }

}
