package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ViperSlide {
    private DcMotor left;
    private DcMotor right;
    private DcMotor pivotLeft;
    private DcMotor pivotRight;
    private static final int MAX_LIMIT = 1000;
    private static final int MIN_LIMIT = 30;

    public ViperSlide(HardwareMap map) {
        left = map.get(DcMotor.class, "viperLeft");
        right = map.get(DcMotor.class, "viperRight");
        pivotLeft = map.get(DcMotor.class, "pivotLeft");
        pivotRight = map.get(DcMotor.class, "pivotRight");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        pivotLeft.setDirection(DcMotor.Direction.REVERSE);
        pivotRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void setViperPowers(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    private void setPivotPowers(double power) {
        pivotLeft.setPower(power);
        pivotRight.setPower(power);
    }

    private void setPositionMode() {
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setContinuousMode() {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setTargetPosition(int position, double power) {
        left.setTargetPosition(position);
        right.setTargetPosition(position);
        setViperPowers(power);
    }


    public int getViperPosition() {
        int currentPosition = (left.getCurrentPosition() + right.getCurrentPosition()) / 2;
        return currentPosition;
    }

    public void extend(double power) {
        if (getViperPosition() < MAX_LIMIT) {
            setViperPowers(power);
        } else {
            stop();
        }
    }

    public void viperRunToPosition(int position, double power) {
        setPositionMode();
        setTargetPosition(position, power);
    }

    public void (int position, double power) {
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setTargetPosition(position);
        right.setTargetPosition(position);

        setViperPowers(power);
    }



    public void retract(double power) {
        if (getViperPosition() > MIN_LIMIT) {
            setViperPowers(-power);
        } else {
            stop();
        }
    }

    public void stop() {
        setViperPowers(0);
    }

    public void raise(double power) {
        setPivotPowers(power);
    }

    public void lower(double power) {
        setPivotPowers(-power);
    }

    public void stopPivot() {
        setPivotPowers(0);
    }

}
