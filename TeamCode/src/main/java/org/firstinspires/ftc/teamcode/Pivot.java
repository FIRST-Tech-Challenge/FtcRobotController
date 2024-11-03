package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Pivot {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private static final double DEFAULT_POWER = 0.8;
    private static final int POSITION_TOLERANCE = 10;
    public static final int MIN_POSITION = -1750;
    public static final int MAX_POSITION = 0;

    private int targetPosition = 0;
    private PivotState currentState = PivotState.STOPPED;

    private enum PivotState {
        MOVING,
        HOLDING,
        STOPPED
    }

    public Pivot(DcMotorEx leftMotor, DcMotorEx rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
    }

    private void setState(PivotState newState, double velocity) {
        // Only change motors if state is different
        if (newState != currentState) {
            switch (newState) {
                case MOVING:
                    if (leftMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    leftMotor.setVelocity(velocity);
                    rightMotor.setVelocity(velocity);
                    break;

                case HOLDING:
                    int currentPos = getCurrentPosition();
                    leftMotor.setTargetPosition(currentPos);
                    rightMotor.setTargetPosition(currentPos);
                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftMotor.setPower(DEFAULT_POWER);
                    rightMotor.setPower(DEFAULT_POWER);
                    break;

                case STOPPED:
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
            }
            currentState = newState;
        }

        // If moving up or down, update power even if already in that state
        else if ((newState == PivotState.MOVING) && velocity != Math.abs(leftMotor.getPower())) {
            leftMotor.setVelocity(velocity);
            rightMotor.setVelocity(velocity);
        }
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double velocity) {
        int currentPosition = getCurrentPosition();
        if (velocity > 0 && currentPosition >= MAX_POSITION || velocity < 0 && currentPosition <= MIN_POSITION) {
            setState(Pivot.PivotState.HOLDING, 0);
            return;
        }
        setState(PivotState.MOVING, velocity);
    }

    public void stop() {
        setState(PivotState.STOPPED, 0);
    }

    public void hold() {
        setState(PivotState.HOLDING, DEFAULT_POWER);
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);

        // Only change mode and power if not already in RUN_TO_POSITION
        if (leftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setPower(DEFAULT_POWER);
            rightMotor.setPower(DEFAULT_POWER);
        }
        currentState = PivotState.HOLDING;
    }

    public boolean isAtTargetPosition() {
        int leftError = Math.abs(leftMotor.getCurrentPosition() - targetPosition);
        int rightError = Math.abs(rightMotor.getCurrentPosition() - targetPosition);
        return leftError < POSITION_TOLERANCE && rightError < POSITION_TOLERANCE;
    }

    public boolean isDrifting() {
        int positionDifference = Math.abs(leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition());
        return positionDifference > POSITION_TOLERANCE;
    }

    public double getAngleDegrees() {
        return -(getCurrentPosition() + 430) / 10.5;
    }

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }

    public PivotState getCurrentState() {
        return currentState;
    }
}
