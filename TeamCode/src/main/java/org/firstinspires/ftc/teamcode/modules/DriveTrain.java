package org.firstinspires.ftc.teamcode.modules;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

public class DriveTrain {
    private final OpModeUtilities opModeUtilities;
    //private final DcMotor testMotorDeleteLater;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

    public DriveTrain(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        fLeft = opModeUtilities.getHardwareMap().dcMotor.get("fLeft");
        fRight = opModeUtilities.getHardwareMap().dcMotor.get("fRight");
        bLeft = opModeUtilities.getHardwareMap().dcMotor.get("bLeft");
        bRight = opModeUtilities.getHardwareMap().dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightEncoder = bRight;
        leftEncoder = bLeft;
        backEncoder = fRight;

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


       // testMotorDeleteLater = opModeUtilities.getHardwareMap().dcMotor.get("testMotor");
    }
    public void setFLeftPower(double power) { fLeft.setPower(power); }

    public void setFRightPower(double power) { fRight.setPower(power); }

    public void setBLeftPower(double power) { bLeft.setPower(power); }

    public void setBRightPower(double power) { bRight.setPower(power); }

    public void setPower (double fLeftPower, double fRightPower, double bLeftPower, double bRightPower){
        setFLeftPower(fLeftPower);
        setFRightPower(fRightPower);
        setBLeftPower(bLeftPower);
        setBRightPower(bRightPower);
        Log.d("purepursaction_power", "power " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);
    }

    public void setPower(double power) {
        setFLeftPower(power);
        setFRightPower(power);
        setBLeftPower(power);
        setBRightPower(power);
    }

    public DcMotor getBackEncoder() {
        return backEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getfLeft() {
        return fLeft;
    }

    public double getfLeftTicks() {
        return fLeft.getCurrentPosition();
    }

    public DcMotor getfRight() {
        return fRight;
    }

    public DcMotor getbLeft() {
        return bLeft;
    }

    public DcMotor getbRight() {
        return bRight;
    }

    /*public DcMotor getTestMotorDeleteLater() {
        return testMotorDeleteLater;
    }*/

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    /*public void setTestPower(double power) throws InterruptedException {
        getTestMotorDeleteLater().setPower(power);
        Thread.sleep(3000);
        getTestMotorDeleteLater().setPower(0);
    }*/
}
