package com.kalipsorobotics.actions.outtake.teleopActions;

import android.util.Log;

import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAction {

    private final Outtake outtake;
    private final OuttakePivotAction outtakePivotAction;
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;
    MoveOuttakeLSAction moveOuttakeLSAction;
    int stage = 0;

    public OuttakeSlideAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakePivotAction = new OuttakePivotAction(outtake);
        this.linearSlideMotor1 = outtake.getLinearSlide1();
        this.linearSlideMotor2 = outtake.getLinearSlide2();
    }
    public void setTargetPos(int position) {
        linearSlideMotor1.setTargetPosition(position);
        linearSlideMotor2.setTargetPosition(position);
    }
    public void setMode(DcMotor.RunMode mode) {
        linearSlideMotor1.setMode(mode);
        linearSlideMotor2.setMode(mode);
    }
    public void setPower(double power) {
        Log.d("outtake slides", "POWER: " + power);
        linearSlideMotor1.setPower(power);
        linearSlideMotor2.setPower(power);
    }
    public void setPower2(double power2) {
        if (power2 > 0) {
            setTargetPos(getPosition() + 3);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (power2 < 0) {
            setTargetPos(getPosition() - 3);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setTargetPos(getPosition());
            stop();
        }
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

    public void reverse() {
        setPower(-1);
    }

    public void moveToPosition(int target) {
        if (moveOuttakeLSAction != null && !moveOuttakeLSAction.checkDoneCondition()) {
            return;
        }
        moveOuttakeLSAction = new MoveOuttakeLSAction(this.outtake, target);
        moveOuttakeLSAction.updateCheckDone();

    }

    public void updateCheckDone() {
        moveOuttakeLSAction.updateCheckDone();
    }


    public void up(double distance) {
        if (getPosition() < distance) {
            setPower(1);
        } else {
            setPower(0);
        }
    }

    public int getPosition() {
        return linearSlideMotor1.getCurrentPosition();
    }

    public void down() {
        if (getPosition() > 0) {
            setPower(-1);
        } else {
            setPower(0.0);
        }
    }

    public void toggle() {
        if (stage == 0) {
            moveToPosition(400);
            stage = 1;
        } else {
            moveToPosition(0);
            stage = 0;
        }
    }
}
