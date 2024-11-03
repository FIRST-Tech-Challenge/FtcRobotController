package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ViperSlide {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    public static final double DEFAULT_POWER = 0.8;
    public static final int POSITION_TOLERANCE = 10;
    public static final int MOVE_COUNTS_PER_INCH = 111;
    public static final double BASE_ARM_LENGTH = 12;
    public static final double CHASSIS_TO_PIVOT_LENGTH = 9;
    public static final int MIN_POSITION = 10;
    public static final int MAX_POSITION = 3000;   // Adjust for your robot
    public static final int MAX_POSITION_HANG = 1320;

    // Preset positions - adjust these for your robot
    public static final int POSITION_GROUND = 0;
    public static final int POSITION_LOW = 1000;
    public static final int POSITION_MEDIUM = 2000;
    public static final int POSITION_HIGH = 2800;

    private int targetPosition = 0;
    private SlideState currentState = SlideState.STOPPED;

    private enum SlideState {
        MOVING,
        HOLDING,
        STOPPED
    }

    public ViperSlide(DcMotorEx leftMotor, DcMotorEx rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Clockwise
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Counter-clockwise
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetEncoders();
    }

    private void setState(SlideState newState, double velocity) {
        // Only change motors if state is different
        if (newState != currentState) {
            switch (newState) {
                case MOVING:
                    if (leftMotor.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    leftMotor.setVelocity(velocity);
                    rightMotor.setVelocity(velocity);
                    break;

                case HOLDING:
                    int currentPos = getCurrentPosition();
                    leftMotor.setTargetPosition(currentPos);
                    rightMotor.setTargetPosition(currentPos);
                    leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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

        // If extending or retracting, update velocity even if already in that state
        else if ((newState == SlideState.MOVING) && velocity != Math.abs(leftMotor.getVelocity())) {
            leftMotor.setVelocity(velocity);
            rightMotor.setVelocity(velocity);
        }
    }

    public double getExtensionBeyondChassis(double angle) {
        double extension = Math.max(Math.abs(Math.cos(Math.toRadians(angle)) * (((double) getCurrentPosition() / MOVE_COUNTS_PER_INCH) + BASE_ARM_LENGTH)) - CHASSIS_TO_PIVOT_LENGTH, 0);
        if (angle > 90) {
            extension *= -1;
        }
        return extension;
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void move(double velocity) {
        int currentPosition = getCurrentPosition();
        if (velocity > 0 && currentPosition >= MAX_POSITION || velocity < 0 && currentPosition <= MIN_POSITION) {
            setState(SlideState.HOLDING, 0);
            return;
        }
        setState(SlideState.MOVING, velocity);
    }

    public void stop() {
        setState(SlideState.STOPPED, 0);
    }

    public void hold() {
        setState(SlideState.HOLDING, DEFAULT_POWER);
    }

    public boolean setTargetPosition(int position) {
        if (position < MIN_POSITION || position > MAX_POSITION) {
            return false;
        }
        targetPosition = position;
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);

        // Only change mode and power if not already in RUN_TO_POSITION
        if (leftMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftMotor.setPower(DEFAULT_POWER);
            rightMotor.setPower(DEFAULT_POWER);
        }
        currentState = SlideState.HOLDING;
        return true;
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

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }

    public boolean isAtBottom() {
        return getCurrentPosition() <= MIN_POSITION;
    }

    public boolean isAtTop() {
        return getCurrentPosition() >= MAX_POSITION;
    }

    public SlideState getCurrentState() {
        return currentState;
    }
}
