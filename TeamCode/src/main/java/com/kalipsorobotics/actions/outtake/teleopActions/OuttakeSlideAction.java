package com.kalipsorobotics.actions.outtake.teleopActions;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OuttakeSlideAction {

    private final Outtake outtake;
    private final OuttakePivotAction outtakePivotAction;
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;
    int stage = 0;

    public OuttakeSlideAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakePivotAction = new OuttakePivotAction(outtake);
        this.linearSlideMotor1 = outtake.getLinearSlideMotor1();
        this.linearSlideMotor2 = outtake.getLinearSlide2();
    }


    public void setPower(double power) {
//        linearSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        if (power > 0.1) {
//            moveToPosition(2040);
//        } else if (power < -0.1) {
//            moveToPosition(0);
//        }
        linearSlideMotor1.setPower(power);
        Log.d("outtake slides", "POWER: " + power);
        linearSlideMotor2.setPower(power);
    }

    public void run() {
        setPower(1);
    }

    public void stop() {
        setPower(0);
    }

    public void reverse() { setPower(-1); }

    public void idle() {
        setPower(0.2);
    }

//    public void moveToPosition(int target) {
//        Log.d("out slide", "MOVING");
//        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setPower(1);
//        linearSlideMotor2.setTargetPosition(target);
//        linearSlideMotor1.setTargetPosition(target);
//        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
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
    public void toggle() {
        if (stage == 0) {
            run();
            SystemClock.sleep(300);
            outtakePivotAction.moveOut();
            stage = 1; }
        else {
            reverse();
            SystemClock.sleep(300);
            stage = 0; }
    }
    public void setL1Power(double power) {
        linearSlideMotor1.setPower(power);
    }
    public void setL2Power(double power) {
        linearSlideMotor2.setPower(power);
    }
}
