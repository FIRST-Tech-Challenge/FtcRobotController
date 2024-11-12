package com.kalipsorobotics.actions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Outtake;

public class MoveLSAction extends Action {

    Outtake outtake;
    DcMotor linearSlide, linearSlideTwo;
    final double ERROR_TOLERANCE = 90;
    final double P_CONSTANT = 0.005;
    double targetTicks;
    double currentTicks;
    double error;

    public MoveLSAction(double targetTicks, Outtake outtake) {
        this.outtake = outtake;
        linearSlide = outtake.linearSlideMotor1;
        linearSlideTwo = outtake.linearSlideMotor2;
        this.dependentAction = new DoneStateAction();
        this.targetTicks = -targetTicks;
    }

    private double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
        Log.d("movels", error + " " + targetTicks + " " + currentTicks);
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        Log.d("movels", "error is " + error);
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            linearSlide.setPower(0);
            linearSlideTwo.setPower(0);
            Log.d("movels", "done");
            //outtake.getOpModeUtilities().getOpMode().sleep(100);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        this.currentTicks = linearSlide.getCurrentPosition();

        if(!hasStarted) {
            Log.d("movels", "started");
            this.targetTicks -= currentTicks;
            hasStarted = true;
        }

        linearSlide.setPower(calculatePower()+0.1);
        linearSlideTwo.setPower(calculatePower()+0.1);
    }
}
