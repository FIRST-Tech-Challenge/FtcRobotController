package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

public class Elevator {

    private LinearOpMode myOpMode = null;
    final double HOME_POWER = -0.1;
    double SLOW_LIFT = 0.2;
    double SLOW_LOWER = -0.1;
    double FAST_LIFT = 0.5;
    double FAST_LOWER = -0.2;
    int DEAD_BAND = 4;

    private DcMotorEx liftMaster;
    private DcMotorEx liftSlave;
    private List<DcMotorEx> motors;

    private int target = 0;
    private int     lastPosition = 0;
    private double  power    = 0;
    private boolean liftActive = false;

    public Elevator(LinearOpMode opMode) {

        myOpMode = opMode;
        liftMaster = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_master");
        liftSlave = myOpMode.hardwareMap.get(DcMotorEx.class, "lefte");
        motors = Arrays.asList(liftMaster, liftSlave);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        liftSlave.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean runControlLoop() {
        double error = target - getPosition();
        if (error > DEAD_BAND*4) {
            setPower(FAST_LIFT);
        }
        else if (error > DEAD_BAND) {
            setPower(SLOW_LIFT);
        }
        else if (error < -DEAD_BAND*4) {
            setPower(FAST_LOWER);
        }
        else if (error < -DEAD_BAND) {
            setPower(SLOW_LOWER);
        }
        else {
            setPower(0);
        }

        return true;
    }

    public void setHome() {
        disableLift();  // Stop any closed loop control
        lastPosition = liftMaster.getCurrentPosition();
        setPower(HOME_POWER);
        myOpMode.sleep(250);

        while (!myOpMode.isStopRequested() && (liftMaster.getCurrentPosition() != lastPosition)){
            lastPosition = liftMaster.getCurrentPosition();
            myOpMode.sleep(100);
        }

        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myOpMode.sleep(50);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTarget(0);
        enableLift();  // Start closed loop control
    }

    public void setPower(double power){
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setTarget(int   target){
        this.target = target;
    }

    public int getTarget(){
        return target;
    }

    public void enableLift() {
        liftActive = true;
    }

    public void disableLift() {
        liftActive = false;
    }

    public int getPosition() {
        return liftMaster.getCurrentPosition();
    }

    public void manualControl() {
        if (myOpMode.gamepad1.dpad_up && getPosition() <= 1000) {
            setPower(SLOW_LIFT);
        }
        else if (myOpMode.gamepad1.dpad_down && getPosition() > 0) {
            setPower(SLOW_LOWER);
        }
        else {
            setPower(0);
        }
    }
}

