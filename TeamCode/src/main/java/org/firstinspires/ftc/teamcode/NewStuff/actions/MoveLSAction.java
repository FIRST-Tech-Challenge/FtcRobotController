package org.firstinspires.ftc.teamcode.NewStuff.actions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NewStuff.actions.code2023.Outtake2023;
import org.firstinspires.ftc.teamcode.NewStuff.modules.Outtake2024;

public class MoveLSAction extends Action {

    Outtake2024 outtake;
    DcMotor linearSlide, linearSlideTwo;
    final double ERROR_TOLERANCE = 50;
    final double P_CONSTANT = 0.0035;
    double targetTicks;
    double currentTicks;
    double error;


    public MoveLSAction(Action dependentAction, double targetTicks, Outtake2024 outtake) {
        this.outtake = outtake;
        linearSlide = outtake.linearSlide;
        linearSlideTwo = outtake.linearSlideTwo;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public MoveLSAction(double targetTicks, Outtake2024 outtake) {
        this.outtake = outtake;
        linearSlide = outtake.linearSlide;
        linearSlideTwo = outtake.linearSlideTwo;
        this.dependentAction = new DoneStateAction();
        this.targetTicks = targetTicks;
    }

    private double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        Log.d("movels", "error is " + error);
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            linearSlide.setPower(0);
            linearSlideTwo.setPower(0);
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
            this.targetTicks += currentTicks;
            hasStarted = true;
        }

        linearSlide.setPower(calculatePower());
        linearSlideTwo.setPower(calculatePower());
    }
}
