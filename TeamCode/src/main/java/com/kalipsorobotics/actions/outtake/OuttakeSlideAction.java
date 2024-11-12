package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAction {

    Outtake outtake;
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;

    public OuttakeSlideAction(Outtake outtake) {
        this.outtake = outtake;
        this.linearSlideMotor1 = outtake.linearSlideMotor1;
        this.linearSlideMotor2 = outtake.linearSlideMotor2;
    }


    public void setPower(double power) {
        linearSlideMotor1.setPower(power);
        linearSlideMotor2.setPower(power);
    }

    public void run() {
        setPower(1);
    }

    public void stop() {
        setPower(0);
    }

    public void idle() {
        setPower(0.1);
    }

    public void moveToPosition() {

    }

}
