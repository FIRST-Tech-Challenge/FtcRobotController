package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAction {

    Outtake outtake;
    DcMotor linearSlide1;
    DcMotor linearSlide2;

    public OuttakeSlideAction(Outtake outtake) {
        this.outtake = outtake;
        this.linearSlide1 = outtake.linearSlide1;
        this.linearSlide2 = outtake.linearSlide2;
    }


    public void setPower(double power) {
        linearSlide1.setPower(power);
        linearSlide2.setPower(power);
    }



    public void moveToPosition() {

    }

}
