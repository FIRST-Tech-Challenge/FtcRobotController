package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;

public class StandardMechanumRobotDrivetrain extends StandardRobotDrivetrain {

    private final StandardMotor RIGHT_TOP, RIGHT_BOTTOM, LEFT_TOP, LEFT_BOTTOM;

    /**
     * Creates a new mechanum drivetrain.
     * @param rightTop The right top motor of the mechanum
     * @param rightBottom The right bottom motor of the mechanum
     * @param leftTop The left top motor of the mechanum
     * @param leftBottom The left bottom motor of the mechanum
     */
    public StandardMechanumRobotDrivetrain(StandardMotor rightTop, StandardMotor rightBottom, StandardMotor leftTop, StandardMotor leftBottom) {
        RIGHT_TOP = rightTop;
        RIGHT_BOTTOM = rightBottom;
        LEFT_TOP = leftTop;
        LEFT_BOTTOM = leftBottom;
    }

    /**
     * Drives the mechanum a distance
     * @param rightTopDistance The distance of the top right motor
     * @param rightBottomDistance The distance of the top right motor
     * @param leftTopDistance The distance of the top left motor
     * @param leftBottomDistance The distance of the bottom motor
     * @param speed The speed of all four motors
     */
    public void driveDistance(int rightTopDistance, int rightBottomDistance, int leftTopDistance, int leftBottomDistance, int speed) {
        RIGHT_TOP.driveDistance(rightTopDistance, speed);
        RIGHT_BOTTOM.driveDistance(rightBottomDistance, speed);
        LEFT_TOP.driveDistance(leftTopDistance, speed);
        LEFT_BOTTOM.driveDistance(leftBottomDistance, speed);
    }

    /**
     * Drives the mechanum at a certain speed
     * @param rightTopSpeed The speed of the top right motor
     * @param rightBottomSpeed The speed of the top right motor
     * @param leftTopSpeed The speed of the top left motor
     * @param leftBottomSpeed The speed of the bottom motor
     */
    public void driveWithEncoder(int rightTopSpeed, int rightBottomSpeed, int leftTopSpeed, int leftBottomSpeed) {
        RIGHT_TOP.driveWithEncoder(rightTopSpeed);
        RIGHT_BOTTOM.driveWithEncoder(rightBottomSpeed);
        LEFT_TOP.driveWithEncoder(leftTopSpeed);
        LEFT_BOTTOM.driveWithEncoder(leftBottomSpeed);
    }

    /**
     * Sends a certain voltage to the mechanum
     * @param rightTopPower The power of the top right motor
     * @param rightBottomPower The power of the top right motor
     * @param leftTopPower The power of the top left motor
     * @param leftBottomPower The power of the bottom motor
     */
    public void driveWithoutEncoder(int rightTopPower, int rightBottomPower, int leftTopPower, int leftBottomPower) {
        RIGHT_TOP.driveWithoutEncoder(rightTopPower);
        RIGHT_BOTTOM.driveWithoutEncoder(rightBottomPower);
        LEFT_TOP.driveWithoutEncoder(leftTopPower);
        LEFT_BOTTOM.driveWithoutEncoder(leftBottomPower);
    }

    /**
     * Brings the mechanum to a stop and resets it
     */
    public void stop() {
        brake();
        reset();
    }

    /**
     * Brings the mechanum to a stop
     */
    public void brake() {
        RIGHT_TOP.brake();
        RIGHT_BOTTOM.brake();
        LEFT_TOP.brake();
        LEFT_BOTTOM.brake();
    }

    /**
     * Resets the mechanum
     */
    public void reset() {
        RIGHT_TOP.reset();
        RIGHT_BOTTOM.reset();
        LEFT_TOP.reset();
        LEFT_BOTTOM.reset();
    }

    public StandardMotor getRightTop() {
        return RIGHT_TOP;
    }

    public StandardMotor getRightBottom() {
        return RIGHT_BOTTOM;
    }

    public StandardMotor getLeftTop() {
        return LEFT_TOP;
    }

    public StandardMotor getLeftBottom() {
        return LEFT_BOTTOM;
    }

    @Override
    public boolean isInputDevice() {
        return true;
    }

    @Override
    public boolean isOutputDevice() {
        return false;
    }
}
