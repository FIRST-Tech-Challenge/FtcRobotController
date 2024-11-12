package com.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Intake;

public class IntakeNoodleAction {

    Intake intake;

    DcMotor intakeNoodleMotor;

    public IntakeNoodleAction(Intake intake) {
        this.intake = intake;
        this.intakeNoodleMotor = intake.getNoodleMotor();
    }


    public void run() {
        intakeNoodleMotor.setPower(1);
    }
    public void reverse() {
        intakeNoodleMotor.setPower(-1);
    }
    public void stop() {
        intakeNoodleMotor.setPower(0);
    }
}
