package com.kalipsorobotics.actions.intake;

import static com.kalipsorobotics.math.CalculateTickPer.MAX_RANGE_INTAKE_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.MIN_RANGE_INTAKE_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.degToTicksIntakeLS;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveIntakeLSAction extends Action {

    public enum Position {
        Down, SPECIMEN, BASKET
    }

    Intake intake;
    private static double globalIntakeSlideMaintainTicks = 0;
    DcMotor intakeSlideMotor;
    final double ERROR_TOLERANCE_TICKS = CalculateTickPer.degToTicksIntakeLS(0.5);
    private final double P_CONSTANT = 10 * (1 / CalculateTickPer.degToTicksIntakeLS(90));
    final double targetTicks;
    private double currentTicks;
    public MoveIntakeLSAction(Intake intake, double targetDeg) {
        this.intake = intake;
        this.intakeSlideMotor = intake.getLinkageMotor();
        this.targetTicks = CalculateTickPer.degToTicksIntakeLS(targetDeg);
        this.dependentActions.add(new DoneStateAction());
    }

    private double calculatePower(double targetError) {
        double power = targetError * P_CONSTANT;
        double lowestPower = 0.2;
//
//        if (globalIntakeSlideMaintainTicks > degToTicksIntakeLS(3)) {
//            lowestPower = 0.1;
//        }

        if (Math.abs(power) < lowestPower) {
            power = power * (lowestPower / Math.abs(power));
        }

        return  power;

    }

    @Override
    public boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    public void update() {

        if(!hasStarted) {
            setGlobalLinearSlideMaintainTicks(targetTicks);
            Log.d("Outtake_LS", "set global to" + globalIntakeSlideMaintainTicks);
            hasStarted = true;
        }

        double currentTargetTicks;
        this.currentTicks = intakeSlideMotor.getCurrentPosition();


        if (isDone) {
            currentTargetTicks = globalIntakeSlideMaintainTicks;
            Log.d("Outtake_LS", "global maintanance pos" + globalIntakeSlideMaintainTicks);
        } else {
            currentTargetTicks = this.targetTicks;
        }


        //soft stop for low and high
        if (currentTargetTicks > MAX_RANGE_INTAKE_TICKS) {
            currentTargetTicks = MAX_RANGE_INTAKE_TICKS;
        } else if (currentTargetTicks < MIN_RANGE_INTAKE_TICKS) {
            currentTargetTicks = MIN_RANGE_INTAKE_TICKS;
        }

        double targetErrorTicks = currentTargetTicks - currentTicks;

        double power = calculatePower(targetErrorTicks);

        if ((Math.abs(this.targetTicks - currentTicks) <= ERROR_TOLERANCE_TICKS) && !isDone) {
            Log.d("Intake_LS", String.format("action done for=%s, targetErrorTicks=%.3f, errorTolerance=%.3f, " +
                            "targetTicks=%.3f, " +
                            "currentTicks=%.3f, ",
                    this.name, targetErrorTicks, ERROR_TOLERANCE_TICKS, targetTicks,
                    currentTicks));
            isDone = true;
        }
        if (Math.abs(targetErrorTicks) < ERROR_TOLERANCE_TICKS) {
            power = 0;
            if (globalIntakeSlideMaintainTicks <= degToTicksIntakeLS(2)) {
                power = 0;
            }
        }
        Log.d("Intake_LS", String.format(
                "Setting power, targetErrorTicks=%.3f, errorTolerance=%.3f, currentTargetTicks=%.3f" +
                        "currentTicks=%.3f, " +
                        "power=%.3f",
                targetErrorTicks, ERROR_TOLERANCE_TICKS, currentTargetTicks,
                currentTicks, power));
        intakeSlideMotor.setPower(power);
    }

    static public void setGlobalLinearSlideMaintainTicks(double pos) {
        Log.d("Outtake_LS setGlobalLinearSlideMaintainTicks", "setGlobal " + pos);
        if (pos < MIN_RANGE_INTAKE_TICKS) {
            globalIntakeSlideMaintainTicks = MIN_RANGE_INTAKE_TICKS;
        } else if (pos > MAX_RANGE_INTAKE_TICKS) {
            globalIntakeSlideMaintainTicks = MAX_RANGE_INTAKE_TICKS;
        } else {
            globalIntakeSlideMaintainTicks = pos;
        }
    }

    static public void incrementGlobal(double incrementTicks) {
        setGlobalLinearSlideMaintainTicks(globalIntakeSlideMaintainTicks + incrementTicks);
    }
    public double getCurrentTicks() {
        return currentTicks;
    }

}
