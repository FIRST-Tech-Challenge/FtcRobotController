package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;

public class StandardTankVehicleDrivetrain extends StandardVehicleDrivetrain {

    private final StandardMotor RIGHT_TOP, RIGHT_BOTTOM, LEFT_TOP, LEFT_BOTTOM;

    /**
     * Creates a new tank drivetrain.
     * @param rightTop The right top motor of the tank
     * @param rightBottom The right bottom motor of the tank
     * @param leftTop The left top motor of the tank
     * @param leftBottom The left bottom motor of the tank
     */
    public StandardTankVehicleDrivetrain(StandardMotor rightTop, StandardMotor rightBottom, StandardMotor leftTop, StandardMotor leftBottom) {
        RIGHT_TOP = rightTop;
        RIGHT_BOTTOM = rightBottom;
        LEFT_TOP = leftTop;
        LEFT_BOTTOM = leftBottom;
    }

    /**
     * Drives the tank a distance
     * @param rightDistance The distance of the right two motors
     * @param leftDistance The distance of the left two motors
     * @param speed The speed of all four motors
     */
    @Override
    public void driveDistance(int rightDistance, int leftDistance, int speed) {
        RIGHT_TOP.driveDistance(rightDistance, speed);
        RIGHT_BOTTOM.driveDistance(rightDistance, speed);
        LEFT_TOP.driveDistance(leftDistance, speed);
        LEFT_BOTTOM.driveDistance(leftDistance, speed);
    }

    /**
     * Drives the tank a distance
     * @param distance The distance of all four motors
     * @param speed The speed of all four motors
     */
    @Override
    public void driveDistance(int distance, int speed) {
        RIGHT_TOP.driveDistance(distance, speed);
        RIGHT_BOTTOM.driveDistance(distance, speed);
        LEFT_TOP.driveDistance(distance, speed);
        LEFT_BOTTOM.driveDistance(distance, speed);
    }

    /**
     * Drives the tank at a certain speed
     * @param rightSpeed The speed of the right two motors
     * @param leftSpeed The speed of the left two motors
     */
    @Override
    public void driveWithEncoder(int rightSpeed, int leftSpeed) {
        RIGHT_TOP.driveWithEncoder(rightSpeed);
        RIGHT_BOTTOM.driveWithEncoder(rightSpeed);
        LEFT_TOP.driveWithEncoder(leftSpeed);
        LEFT_BOTTOM.driveWithEncoder(leftSpeed);
    }

    /**
     * Drives the tank at a certain speed
     * @param speed The speed of all four motors
     */
    @Override
    public void driveWithEncoder(int speed) {
        RIGHT_TOP.driveWithEncoder(speed);
        RIGHT_BOTTOM.driveWithEncoder(speed);
        LEFT_TOP.driveWithEncoder(speed);
        LEFT_BOTTOM.driveWithEncoder(speed);
    }

    /**
     * Sends a certain voltage to the tank
     * @param rightPower The voltage to send to the right two motors
     * @param leftPower The voltage to send to the left two motors
     */
    @Override
    public void driveWithoutEncoder(int rightPower, int leftPower) {
        RIGHT_TOP.driveWithoutEncoder(rightPower);
        RIGHT_BOTTOM.driveWithoutEncoder(rightPower);
        LEFT_TOP.driveWithoutEncoder(leftPower);
        LEFT_BOTTOM.driveWithoutEncoder(leftPower);
    }

    /**
     * Sends a certain voltage to the tank
     * @param power The voltage to send
     */
    @Override
    public void driveWithoutEncoder(int power) {
        RIGHT_TOP.driveWithoutEncoder(power);
        RIGHT_BOTTOM.driveWithoutEncoder(power);
        LEFT_TOP.driveWithoutEncoder(power);
        LEFT_BOTTOM.driveWithoutEncoder(power);
    }

    /**
     * Brings the tank to a stop and resets it
     */
    @Override
    public void stop() {
        brake();
        reset();
    }

    /**
     * Brings the tank to a stop
     */
    @Override
    public void brake() {
        RIGHT_TOP.brake();
        RIGHT_BOTTOM.brake();
        LEFT_TOP.brake();
        LEFT_BOTTOM.brake();
    }

    /**
     * Resets the tank
     */
    @Override
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
