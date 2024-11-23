package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAction {

    Outtake outtake;
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;

    public OuttakeSlideAction(Outtake outtake) {
        this.outtake = outtake;
        this.linearSlideMotor1 = outtake.getLinearSlideMotor1();
        this.linearSlideMotor2 = outtake.getLinearSlide2();
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
    public boolean toggle() {
        if (getPosition() < 920) {
            while (true) {
                setPower(0.9);
                if (getPosition() > 920) {
                    setPower(0);
                    return true;
                }
            }
        }
        else {
            while (true) {
                setPower(-0.9);
                if (getPosition() < 5) {
                    setPower(0);
                    return true;
                }
            }
        }
    }
    public int getPosition() {
        return linearSlideMotor1.getCurrentPosition();
    }
}
