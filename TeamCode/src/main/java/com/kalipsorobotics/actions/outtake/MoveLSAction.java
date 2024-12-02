package com.kalipsorobotics.actions.outtake;

import static com.kalipsorobotics.math.CalculateTickPer.MAX_RANGE_LS_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.MIN_RANGE_LS_TICKS;

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

    private static double globalLinearSlideMaintainTicks = 0;
    DcMotor linearSlide1, linearSlide2;
    final double ERROR_TOLERANCE_TICKS = CalculateTickPer.mmToTicksLS(5);
    double P_CONSTANT = (1 / CalculateTickPer.mmToTicksLS(400.0 * (1.0 / 3.0)));
    final double targetTicks;
    private double currentTicks;
    public MoveLSAction(Outtake outtake, double targetMM) {
        this.outtake = outtake;
        linearSlide1 = outtake.linearSlide1;
        linearSlide2 = outtake.linearSlide2;
        this.targetTicks = CalculateTickPer.mmToTicksLS(targetMM);
        this.dependentActions.add(new DoneStateAction());
    }

    private double calculatePower(double targetError) {
        double power = targetError * P_CONSTANT;
        double lowestPower = 0.15;

        if (globalLinearSlideMaintainTicks > 1800) {
            lowestPower = 0.35;
        }

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
            setGlobalLinearSlideMaintainTicks(targetTicks);
            Log.d("Outtake_LS", "set global to" + globalLinearSlideMaintainTicks);
            hasStarted = true;
        }

        double currentTargetTicks;
        this.currentTicks = linearSlide1.getCurrentPosition();


        if (isDone) {
            currentTargetTicks = globalLinearSlideMaintainTicks;
            Log.d("Outtake_LS", "global maintanance pos" + globalLinearSlideMaintainTicks);
        } else {
            currentTargetTicks = this.targetTicks;
        }


        //soft stop for low and high
        if (currentTargetTicks > MAX_RANGE_LS_TICKS) {
            currentTargetTicks = MAX_RANGE_LS_TICKS;
        } else if (currentTargetTicks < MIN_RANGE_LS_TICKS) {
            currentTargetTicks = MIN_RANGE_LS_TICKS;
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
            power = 0;
            if (globalLinearSlideMaintainTicks < 0) {
                power = 0;
            }
        }
        Log.d("Outtake_LS", String.format(
                "Setting power, targetErrorTicks=%.3f, errorTolerance=%.3f, currentTargetTicks=%.3f" +
                        "currentTicks=%.3f, " +
                        "power=%.3f",
                targetErrorTicks, ERROR_TOLERANCE_TICKS, currentTargetTicks,
                currentTicks, power));
        linearSlide1.setPower(power);
        linearSlide2.setPower(power);
    }

    static public void setGlobalLinearSlideMaintainTicks(double pos) {
        Log.d("Outtake_LS setGlobalLinearSlideMaintainTicks", "setGlobal " + pos);
        if (pos < MIN_RANGE_LS_TICKS) {
            globalLinearSlideMaintainTicks = MIN_RANGE_LS_TICKS;
        } else if (pos > MAX_RANGE_LS_TICKS) {
            globalLinearSlideMaintainTicks = MAX_RANGE_LS_TICKS;
        } else {
            globalLinearSlideMaintainTicks = pos;
        }
    }

    static public void incrementGlobal(double incrementTicks) {
        setGlobalLinearSlideMaintainTicks(globalLinearSlideMaintainTicks + incrementTicks);
    }
    public double getCurrentTicks() {
        return currentTicks;
    }
}
