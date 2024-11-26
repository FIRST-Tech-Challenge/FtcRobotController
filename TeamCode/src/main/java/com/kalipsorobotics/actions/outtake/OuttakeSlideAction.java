package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAction {

    Outtake outtake;
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;
    int stage = 0;

    public OuttakeSlideAction(Outtake outtake) {
        this.outtake = outtake;
        OuttakePivotAction outtakePivotAction = new OuttakePivotAction(outtake);
        this.linearSlideMotor1 = outtake.getLinearSlideMotor1();
        this.linearSlideMotor2 = outtake.getLinearSlide2();
    }


    public void setPower(double power) {
        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void moveToPosition(int target) {
        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(0.75);
        linearSlideMotor2.setTargetPosition(target);
        linearSlideMotor1.setTargetPosition(target);
        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void up(double distance) {
        if (getPosition() < distance) {
            setPower(1);
        }
        else {
            setPower(0);
        }
    }
    public int getPosition() {
        return linearSlideMotor1.getCurrentPosition();
    }
    public void down() {
        if (getPosition() > 0) {
            setPower(-1);
        }
        else {
            setPower(0.0);
        }
    }
    public void Toggle() {
        if (stage == 0) { moveToPosition(992); outtakePivotAction.moveOut(); stage = 1; }
        else { moveToPosition(2060); stage = 2; }
    }
}
