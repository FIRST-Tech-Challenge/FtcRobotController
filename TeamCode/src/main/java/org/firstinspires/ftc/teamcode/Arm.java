package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final DcMotor left;
    private final DcMotor right;
    private final DcMotor pivotLeft;
    private final DcMotor pivotRight;
    public static final int MAX_LIMIT = 3000;
    public static final int MIN_LIMIT = 20;

    public Arm(HardwareMap map) {
        left = map.get(DcMotor.class, "VIPERLEFT");
        right = map.get(DcMotor.class, "VIPERRIGHT");
        pivotLeft = map.get(DcMotor.class, "PIVOTLEFT");
        pivotRight = map.get(DcMotor.class, "PIVOTRIGHT");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        Log.d("Viper", "Right Viper getPower(): " + right.getPower());
        right.setPower(power);
    }

    private void setPivotPowers(double power) {
        Log.d("Arm", "Pivot Power:" + power);
        pivotLeft.setPower(power);
        pivotRight.setPower(power);
    }

    public void setPivotToPositionMode() {
        pivotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPivotToContinuousMode() {
        pivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setViperToPositionMode() {
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setViperToContinuousMode() {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setViperTargetPosition(int position, double power) {
        position = Math.max(Math.min(position, MAX_LIMIT), MIN_LIMIT);
        left.setTargetPosition(position);
        right.setTargetPosition(position);
        setViperToPositionMode();
        setViperPowers(power);
    }

    public void setPivotTargetPosition(int position, double power) {
        Log.d("Arm", "Pivot Target Position: " + position);
        pivotLeft.setTargetPosition(position);
        pivotRight.setTargetPosition(position);
        setPivotToPositionMode();
        setPivotPowers(power);
    }

    public DcMotor.RunMode getViperMode() {
        return left.getMode();
    }

    public DcMotor.RunMode getPivotMode() {
        return pivotLeft.getMode();
    }

    public int getViperCurrentPosition() {
        return (left.getCurrentPosition() + right.getCurrentPosition()) / 2;
    }

    public int getPivotCurrentPosition() {
        return (pivotLeft.getCurrentPosition() + pivotRight.getCurrentPosition()) / 2;
    }

    public int getViperTargetPosition() {
        return (left.getTargetPosition() + right.getTargetPosition()) / 2;
    }

    public int getLeftPivotCurrentPosition() {
//        int currentPosition = (pivotLeft.getCurrentPosition() + pivotRight.getCurrentPosition()) / 2;
        return pivotLeft.getCurrentPosition();
    }

    public int getRightPivotCurrentPosition() {
        return pivotRight.getCurrentPosition();
    }

    public int getLeftPivotTargetPosition() {
//        int currentPosition = (pivotLeft.getCurrentPosition() + pivotRight.getCurrentPosition()) / 2;
        return pivotLeft.getTargetPosition();
    }

    public int getRightPivotTargetPosition() {
        return pivotRight.getTargetPosition();
    }

    public int getPivotTargetPosition() {
        return (pivotLeft.getTargetPosition() + pivotRight.getTargetPosition()) / 2;
    }

    public void moveViperSlides(double power) {
        if (!(getViperCurrentPosition() > MAX_LIMIT && power > 0 || getViperCurrentPosition() < MIN_LIMIT && power < 0)) {
            setViperPowers(power);
        } else {
            stopVipers();
        }
    }

    public void movePivot(double power) {
        setPivotPowers(power);
    }

    public void stopVipers() {
        setViperPowers(0);
    }

    public void stopPivot() {
        setPivotPowers(0);
    }

}
