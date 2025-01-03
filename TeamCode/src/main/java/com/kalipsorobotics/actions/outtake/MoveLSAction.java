package com.kalipsorobotics.actions.outtake;

import static com.kalipsorobotics.math.CalculateTickPer.MAX_RANGE_LS_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.MIN_RANGE_LS_TICKS;
import static com.kalipsorobotics.math.CalculateTickPer.mmToTicksLS;

import android.util.Log;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveLSAction extends Action {

    Outtake outtake;

    private static double globalLinearSlideMaintainTicks = 0;
    DcMotor linearSlide1, linearSlide2;
    public static double ERROR_TOLERANCE_TICKS;
    private final PidNav pidOuttakeLS;
    private double targetTicks;
    private double currentTicks;
    private double overridePower = 0;
    private double targetErrorTicks;

    private double lastTicks;
    private double lastMilli = 0;
    ElapsedTime timeoutTimer;

    private double velocity;
    private boolean overrideOn = false;

    private double startingError; //gives direction of intended travel direction when doing maintenance for hang

    public MoveLSAction(Outtake outtake) {
        this.outtake = outtake;
        linearSlide1 = outtake.linearSlide1;
        linearSlide2 = outtake.linearSlide2;
        this.targetTicks = 0;
        this.pidOuttakeLS = null;
        this.timeoutTimer = new ElapsedTime();
    }

    public MoveLSAction(Outtake outtake, double targetMM) {
        ERROR_TOLERANCE_TICKS = CalculateTickPer.mmToTicksLS(5); // todo move out of constructor
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

    public MoveLSAction(Outtake outtake, double targetMM, double p) {
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

    private double calculatePower(double targetError) {
        double power = pidOuttakeLS.getPower(targetError);

        double lowestPower = 0.08;
        if(Math.abs(targetError) < ERROR_TOLERANCE_TICKS) {
            lowestPower = 0;
            return lowestPower;
        }

        if (targetTicks < mmToTicksLS(30)) {
            lowestPower = 0.3;
        }

        if (targetTicks < mmToTicksLS(15)) {
            lowestPower = 0.15;
        }

        if (targetTicks > mmToTicksLS(100)) {
            lowestPower = 0.13;
        }

        if (targetTicks > mmToTicksLS(400)) {
            lowestPower = 0.15;
        }

        if (targetTicks > mmToTicksLS(650)) {
            lowestPower = 0.25;
        }

        if (targetTicks > mmToTicksLS(700)) {
            lowestPower = 0.23;
        }

        if (targetTicks > mmToTicksLS(720)) {
            lowestPower = 0.13;
        }

        if (Math.abs(power) < lowestPower) {
            power = power * (lowestPower / Math.abs(power));
        }
        return  power;
        //return 1;
    }

    public void finish() {
        isDone = true;
        setLSPower(0);
    }

    public void finishWithoutSetPower() {
        isDone = true;
    }


    public void setOverridePower(double power) {
        Log.d("Outtake_LS", "override power is set to " + power);
        overridePower = power;
        Log.d("Outtake_LS", "override power is " + overridePower);
    }

    public void setOverrideOn(boolean isOverrideOn) {
        overrideOn = isOverrideOn;
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

    public static double getGlobalLinearSlideMaintainTicks() {
        return globalLinearSlideMaintainTicks;
    }

    public void setTargetTicks(double targetTicks) {
        this.targetTicks = targetTicks;
    }

    @Override
    protected boolean checkDoneCondition() {
        if(isDone) {
            return true;
        }

        if ((Math.abs(this.targetTicks - currentTicks) <= ERROR_TOLERANCE_TICKS)) {
            Log.d("Outtake_LS", String.format("action done for=%s, targetErrorTicks=%.3f, errorTolerance=%.3f, " +
                            "targetTicks=%.3f, " +
                            "currentTicks=%.3f, ",
                    this.name, targetErrorTicks, ERROR_TOLERANCE_TICKS, targetTicks,
                    currentTicks));
            finish();
            return true;
        }

        if(velocity < 0.01) {
            if(timeoutTimer.milliseconds() > 10000) {
                finish();
                return true;
            }
        } else {
            timeoutTimer.reset();
        }

        return false;
    }

    @Override
    protected void update() {
        if(isDone) {
            return;
        }

        if(!hasStarted) {
            setGlobalLinearSlideMaintainTicks(targetTicks);
            Log.d("Outtake_LS", "name " + getName() + " set global to " + globalLinearSlideMaintainTicks);
            hasStarted = true;
            lastTicks = linearSlide1.getCurrentPosition();
            timeoutTimer.reset();
            startingError = targetTicks - linearSlide1.getCurrentPosition();
        }

        this.currentTicks = linearSlide1.getCurrentPosition();

        Log.d("Outtake_LS", "global maintanance pos" + globalLinearSlideMaintainTicks);

        //soft stop for low and high
        if (targetTicks > MAX_RANGE_LS_TICKS) {
            targetTicks = MAX_RANGE_LS_TICKS;
        } else if (targetTicks < MIN_RANGE_LS_TICKS) {
            targetTicks = MIN_RANGE_LS_TICKS;
        }

        targetErrorTicks = targetTicks - currentTicks;

        double power = calculatePower(targetErrorTicks);

        Log.d("Outtake_LS", String.format(
                "Setting power, %s targetErrorTicks=%.3f, errorTolerance=%.3f, targetTicks=%.3f" +
                        "currentTicks=%.3f, " +
                        "power=%.3f",
                name, targetErrorTicks, ERROR_TOLERANCE_TICKS, targetTicks,
                currentTicks, power));

        Log.d("Outtake_LS", "before overriding power, override power is " + overridePower);
        if (overrideOn){
            if (power < 0) { //goes down
                power = overridePower * (-1);
            }
            if (power > 0){ //goes up
                power = overridePower;
            }
            Log.d("Outtake_LS", "override power is:" + overridePower + ", power is:" + power);
        }

        velocity = (Math.abs(lastTicks - currentTicks)) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));

        lastMilli = timeoutTimer.milliseconds();
        lastTicks = currentTicks;

        //brake();
        setLSPower(power);
    }

    private void setLSPower(double power) {
        linearSlide1.setPower(power);
        linearSlide2.setPower(power);
        Log.d("Outtake_LS_power", name + " final power set to " + power);
    }
}
