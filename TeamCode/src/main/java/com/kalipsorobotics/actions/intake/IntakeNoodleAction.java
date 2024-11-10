package com.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Intake;

public class IntakeNoodleAction {

    Intake intake;

    DcMotor intakeMotor;

    public IntakeNoodleAction(Intake intake) {
        this.intake = intake;
        this.intakeMotor = intake.getNoodleMotor();
    }


    public void run() {
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
