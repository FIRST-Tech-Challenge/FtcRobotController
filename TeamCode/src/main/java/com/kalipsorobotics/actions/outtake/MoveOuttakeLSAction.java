package com.kalipsorobotics.actions.outtake;

import static com.kalipsorobotics.math.CalculateTickPer.MAX_RANGE_LS_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.MIN_RANGE_LS_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.mmToTicksLS;

import android.util.Log;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.math.Position;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveOuttakeLSAction extends Action {

    public enum Position {
        Down, SPECIMEN, BASKET
    }

    Outtake outtake;

    private static double globalLinearSlideMaintainTicks = 0;
    DcMotor linearSlide1, linearSlide2;
    public static double ERROR_TOLERANCE_TICKS;
    private final PidNav pidOuttakeLS;
    final double targetTicks;
    private double currentTicks;
    private double overridePower = 0;

    private double lastTicks;
    private double lastMilli = 0;
    ElapsedTime timeoutTimer;

    private double velocity;


    public MoveOuttakeLSAction(Outtake outtake) {
        this.outtake = outtake;
        linearSlide1 = outtake.linearSlide1;
        linearSlide2 = outtake.linearSlide2;
        this.targetTicks = 0;
        this.pidOuttakeLS = null;
        this.timeoutTimer = new ElapsedTime();
    }
    public MoveOuttakeLSAction(Outtake outtake, double targetMM) {
        ERROR_TOLERANCE_TICKS =  CalculateTickPer.mmToTicksLS(5);
        double P_CONSTANT = 8 * (1 / CalculateTickPer.mmToTicksLS(400.0));
        this.outtake = outtake;
        linearSlide1 = outtake.linearSlide1;
        linearSlide2 = outtake.linearSlide2;
        this.pidOuttakeLS = new PidNav(P_CONSTANT, 0, 0);
        this.targetTicks = CalculateTickPer.mmToTicksLS(targetMM);
        if (targetTicks >= MAX_RANGE_LS_TICKS) {
            Log.e("Outtake_LS", "target over range, target ticks: " + targetTicks + ", target mm: " + targetMM + ", max: " + MAX_RANGE_LS_TICKS);
        }
        this.dependentActions.add(new DoneStateAction());
        this.timeoutTimer = new ElapsedTime();
    }

    public MoveOuttakeLSAction(Outtake outtake, double targetMM, double p) {
        ERROR_TOLERANCE_TICKS = CalculateTickPer.mmToTicksLS(5);
        double P_CONSTANT = p;
        this.outtake = outtake;
        linearSlide1 = outtake.linearSlide1;
        linearSlide2 = outtake.linearSlide2;
        this.pidOuttakeLS = new PidNav(P_CONSTANT, 0, 0);
        this.targetTicks = CalculateTickPer.mmToTicksLS(targetMM);
        if (targetTicks >= MAX_RANGE_LS_TICKS) {
            Log.e("Outtake_LS", "target over range, target ticks: " + targetTicks + ", target mm: " + targetMM + ", max: " + MAX_RANGE_LS_TICKS);
        }
        this.dependentActions.add(new DoneStateAction());
        this.timeoutTimer = new ElapsedTime();
    }

    public void setPConstant(double P) {
        pidOuttakeLS.setP(P);
    }

    public void setPConstantToDefault() {
        pidOuttakeLS.setP(1 / CalculateTickPer.mmToTicksLS(400.0 * (1.0 / 5.0)));
    }

    private double calculatePower(double targetError) {
        double power = pidOuttakeLS.getPower(targetError);
        double lowestPower = 0.12;

        if (globalLinearSlideMaintainTicks < mmToTicksLS(30)) {
            lowestPower = 0.3;
        }

        if (globalLinearSlideMaintainTicks < mmToTicksLS(15)) {
            lowestPower = 0.45;
        }

        if (globalLinearSlideMaintainTicks > mmToTicksLS(100)) {
            lowestPower = 0.18;
        }

        if (globalLinearSlideMaintainTicks > mmToTicksLS(400)) {
            lowestPower = 0.2;
        }

        if (globalLinearSlideMaintainTicks > mmToTicksLS(650)) {
            lowestPower = 0.25;
        }

        if (globalLinearSlideMaintainTicks > mmToTicksLS(700)) {
            lowestPower = 0.28;
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
            Log.d("Outtake_LS", "name " + getName() + " set global to " + globalLinearSlideMaintainTicks);
            hasStarted = true;
            lastTicks = linearSlide1.getCurrentPosition();
            timeoutTimer.reset();
        }

        double currentTargetTicks;
        this.currentTicks = linearSlide1.getCurrentPosition();

        Log.d("Outtake_LS", "global maintanance pos" + globalLinearSlideMaintainTicks);

        if (isDone) {
            currentTargetTicks = globalLinearSlideMaintainTicks;
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
             if (globalLinearSlideMaintainTicks > (MAX_RANGE_LS_TICKS*0.8)) {
                 power = 0.16;
            } else if (globalLinearSlideMaintainTicks > (MAX_RANGE_LS_TICKS*0.5)){
                 power = 0.15;
             } else if (globalLinearSlideMaintainTicks > 0){
                 power = 0.1;
             } else if (globalLinearSlideMaintainTicks == 0){
                power = 0;
             } else if (globalLinearSlideMaintainTicks < 0) {
                 power = -0.1;
             }

        }

        Log.d("Outtake_LS", String.format(
                "Setting power, %s targetErrorTicks=%.3f, errorTolerance=%.3f, currentTargetTicks=%.3f" +
                        "currentTicks=%.3f, " +
                        "power=%.3f",
                name, targetErrorTicks, ERROR_TOLERANCE_TICKS, currentTargetTicks,
                currentTicks, power));

        Log.d("Outtake_LS", "before overriding power, override power is " + overridePower);
        if (overridePower != 0){
            if (power < 0) { //goes down
                power = overridePower * (-1);
                Log.d("Outtake_LS", "power overrided to " + power);
            }

            if (power > 0){ //goes up
                power = overridePower;
            }
        }

        velocity = (Math.abs(lastTicks - currentTicks)) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));

        if(velocity < 0.0075 && !isDone) {
            if(timeoutTimer.milliseconds() > 1000) {
                isDone = true;
            }
        } else {
            timeoutTimer.reset();
        }

        lastMilli = timeoutTimer.milliseconds();
        lastTicks = currentTicks;

        //brake();
        Log.d("Outtake_LS", "power set to " + power);
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

    static public void setGlobal(double ticks) {
        setGlobalLinearSlideMaintainTicks(ticks);
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public void setI() {
        pidOuttakeLS.setErrorIntegral(0);
    }

    public void setOverridePower(double power) {
        Log.d("Outtake_LS", "override power is set to " + power);
        overridePower = power;
        Log.d("Outtake_LS", "override power is " + overridePower);
    }
}
