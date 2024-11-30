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
    final double ERROR_TOLERANCE_TICKS = 30;
    double P_CONSTANT = 1 / CalculateTickPer.mmToTicksLS(400);
    final double targetTicks;
    double currentTicks;
    final double MIN_IDLE_POWER = 0.15;

    public MoveLSAction(Outtake outtake, double targetMM) {
        this.outtake = outtake;
        linearSlide = outtake.linearSlideMotor1;
        linearSlideTwo = outtake.linearSlideMotor2;
        this.targetTicks = CalculateTickPer.mmToTicksLS(targetMM);
        this.dependentActions.add(new DoneStateAction());
    }

    private double calculatePower(double targetError) {
        double power = targetError * P_CONSTANT;
        double lowestPower = 0.7;
        if (Math.abs(power) < lowestPower) {
            power = power * (lowestPower / Math.abs(power));
        }
        return  power;
        //return 1;
    }

    @Override
    public boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }
        this.currentTicks = linearSlide.getCurrentPosition();
        double targetErrorTicks = targetTicks - currentTicks;


        if(!hasStarted) {
            targetErrorTicks = targetTicks - currentTicks;
            hasStarted = true;
            double power = calculatePower(targetErrorTicks);
            linearSlide.setPower(power);
            linearSlideTwo.setPower(power);
            Log.d("Outtake_LS", String.format("targetDeltaTicks=%.3f, targetTicks=%.3f, currentTicks=%.3f, power=%.3f",
                    targetErrorTicks, targetTicks,
                    currentTicks, power));
        } else if (Math.abs(targetErrorTicks) <= ERROR_TOLERANCE_TICKS) {
            Log.d("Outtake_LS", "done");
            linearSlide.setPower(MIN_IDLE_POWER);
            linearSlideTwo.setPower(MIN_IDLE_POWER);
            isDone = true;
        }

    }
}
