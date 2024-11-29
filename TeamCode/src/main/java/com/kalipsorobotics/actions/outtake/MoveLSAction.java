package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Outtake;

public class MoveLSAction extends Action {

    public enum Position {
        DOWN, SPECIMEN, BASKET
    }


    Outtake outtake;
    DcMotor linearSlide, linearSlideTwo;
    final double ERROR_TOLERANCE = 30;
    double P_CONSTANT = 0.008;
    double targetTicks;
    double currentTicks;
    double error;

    public MoveLSAction(Outtake outtake, double targetMM) {
        this.outtake = outtake;
        linearSlide = outtake.linearSlideMotor1;
        linearSlideTwo = outtake.linearSlideMotor2;
        this.targetTicks = CalculateTickPer.mmToTicksLS(targetMM);
        Log.d("movels", "target ticks set to " + targetMM);
        this.dependentActions.add(new DoneStateAction());
    }

    public MoveLSAction(Outtake outtake, double targetTicks, double P_CONSTANT) {
        this.outtake = outtake;
        linearSlide = outtake.linearSlideMotor1;
        linearSlideTwo = outtake.linearSlideMotor2;
        this.targetTicks = targetTicks;
        Log.d("movels", "target ticks set to " + targetTicks);
        this.dependentActions.add(new DoneStateAction());
        this.P_CONSTANT = P_CONSTANT;
    }

    private double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
        Log.d("movels", "error" + error + " target" + targetTicks + "current " + currentTicks);
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        Log.d("movels", "error is " + error);
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            linearSlide.setPower(Outtake.LS_STAYUP_POWER);
            linearSlideTwo.setPower(Outtake.LS_STAYUP_POWER);
            Log.d("movels", "done");
            //outtake.getOpModeUtilities().getOpMode().sleep(100);
            isDone = true;
            return isDone;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }
        this.currentTicks = linearSlide.getCurrentPosition();

        if(!hasStarted) {
            Log.d("movels", "started");
            this.targetTicks = currentTicks + targetTicks;
            Log.d("movels", "new target is " + targetTicks);
            hasStarted = true;
        }

        linearSlide.setPower(calculatePower());
        linearSlideTwo.setPower(calculatePower());
    }
}
