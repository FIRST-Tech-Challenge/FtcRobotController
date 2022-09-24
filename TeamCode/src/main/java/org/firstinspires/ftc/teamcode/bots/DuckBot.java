
package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DuckBot extends TapeMeasureBot{
    public DcMotor duckSpinner = null;

    boolean isSpinning = false;
    boolean shouldTimeSpin = false;

    int count = 0;

    public double currentSpinnerSpeed = 1;

    ElapsedTime duckTimer = new ElapsedTime(2000);

    //change these values to control what speed the spinner spins around
    public double setSpinnerSpeed = 0.4; //385

    public DuckBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        duckSpinner = hwMap.get(DcMotor.class, "duckArm");
        duckSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void toggleSpinner(double power, boolean isForward) {
        setSpinnerSpeed = power;
        if (isForward) {
            duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            duckSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (!isSpinning) {
            isSpinning = true;
        } else {
            isSpinning = false;
        }
    }

    public void holdSpinner (boolean button, boolean blue, double power) {
        setSpinnerSpeed = power;
        if (button) {
            isSpinning = true;
        } else {
            duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            isSpinning = false;
        }
        if (blue) {
            duckSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void spinCarousel(boolean button) {
        if (button && isSpinning) {
            duckSpinner.setPower(-setSpinnerSpeed);
        } else {
            duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            duckSpinner.setPower(0);
        }
    }

    public void timedSpin(boolean button, double power) {
        setSpinnerSpeed = power;
        if (button) {
            duckTimer.reset();
            isSpinning = true;
        } else if (duckTimer.milliseconds() > 1500) {
            isSpinning = false;
        }
    }

    public void bigTimedSpin(boolean start, boolean stop, double power, boolean blue) {
        setSpinnerSpeed = power;
        if (blue) {
            duckSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (start) {
            duckTimer.reset();
            shouldTimeSpin = true;
        } else if (duckTimer.milliseconds() > 29250 || stop){
            shouldTimeSpin = false;
            isSpinning = false;
            count = 0;
        }
        if (shouldTimeSpin) {
            if (duckTimer.milliseconds() > count && !isSpinning) {
                count += 1500;
                isSpinning = true;
            } else if (duckTimer.milliseconds() > count && isSpinning) {
                count += 750;
                isSpinning = false;
            }
        }
    }

    protected void onTick(){
        spinCarousel(true);
        super.onTick();
    }
}
