package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Outtake;

public class MoveLSAction extends Action {

    public enum Position {
         Down, SPECIMEN, BASKET
    }


    Outtake outtake;

    public static double globalLinearSlideMaintainPos;
    DcMotor linearSlide1, linearSlide2;
    final double ERROR_TOLERANCE_TICKS = CalculateTickPer.mmToTicksLS(10);
    double P_CONSTANT = (1 / CalculateTickPer.mmToTicksLS(400.0 * (1.0 / 3.0)));
    final double targetTicks;
    double currentTicks;
    public MoveLSAction(Outtake outtake, double targetMM) {
        this.outtake = outtake;
        linearSlide1 = outtake.linearSlide1;
        linearSlide2 = outtake.linearSlide2;
        this.targetTicks = CalculateTickPer.mmToTicksLS(targetMM);
        this.dependentActions.add(new DoneStateAction());
    }

    private double calculatePower(double targetError) {
        double power = targetError * P_CONSTANT;
        double lowestPower = 0.4;
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

        if(!hasStarted) {
            globalLinearSlideMaintainPos = targetTicks;
            Log.d("Outtake_LS", "set global to" + globalLinearSlideMaintainPos);
            hasStarted = true;
        }

        double currentTargetTicks;
        this.currentTicks = linearSlide1.getCurrentPosition();

        if (isDone) {
            currentTargetTicks = globalLinearSlideMaintainPos;
            Log.d("Outtake_LS", "global maintanance pos" + globalLinearSlideMaintainPos);
        } else {
            currentTargetTicks = this.targetTicks;
        }

        double targetErrorTicks = currentTargetTicks - currentTicks;

        double power = calculatePower(targetErrorTicks);

        if ((Math.abs(this.targetTicks - currentTicks) <= ERROR_TOLERANCE_TICKS) && !isDone) {
            Log.d("Outtake_LS", String.format("action done for=%s, targetErrorTicks=%.3f, errorTolerance=%.3f, " +
                            "targetTicks=%.3f, " +
                            "currentTicks=%.3f, ",
                    this.name, targetErrorTicks, ERROR_TOLERANCE_TICKS, targetTicks,
                    currentTicks));
            isDone = true;
        }
        if (Math.abs(targetErrorTicks) < ERROR_TOLERANCE_TICKS) {
            power = 0.15;
            if (globalLinearSlideMaintainPos == 0) {
                power = 0;
            }
        }
        Log.d("Outtake_LS", String.format(
                "Setting power, targetErrorTicks=%.3f, errorTolerance=%.3f, targetTicks=%.3f" +
                        "currentTicks=%.3f, " +
                        "power=%.3f",
                targetErrorTicks, ERROR_TOLERANCE_TICKS, targetTicks,
                currentTicks, power));
        linearSlide1.setPower(power);
        linearSlide2.setPower(power);
    }
}
