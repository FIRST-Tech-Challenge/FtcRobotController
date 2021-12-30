package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;

public class StandardCarfaxVehicleDrivetrain extends StandardVehicleDrivetrain {

    private final StandardMotor RIGHT, LEFT;

    /**
     * Creates a new carfax drivetrain.
     * @param right The right top motor of the carfax
     */
    public StandardCarfaxVehicleDrivetrain(StandardMotor right, StandardMotor left) {
        RIGHT = right;
        LEFT = left;
    }

    /**
     * Drives the carfax a distance
     * @param rightDistance The distance of the right motor
     * @param leftDistance The distance of the left motor
     * @param speed The speed of both motors
     */
    public void driveDistance(int rightDistance, int leftDistance, int speed) {
        RIGHT.driveDistance(rightDistance, speed);
        LEFT.driveDistance(leftDistance, speed);
    }

    /**
     * Drives the carfax a distance
     * @param distance The distance of both motors
     * @param speed The speed of both motors
     */
    public void driveDistance(int distance, int speed) {
        RIGHT.driveDistance(distance, speed);
        LEFT.driveDistance(distance, speed);
    }

    /**
     * Drives the carfax at a certain speed
     * @param rightSpeed The speed of the right motor
     * @param leftSpeed The speed of the left motor
     */
    public void driveWithEncoder(int rightSpeed, int leftSpeed) {
        RIGHT.driveWithEncoder(rightSpeed);
        LEFT.driveWithEncoder(leftSpeed);
    }

    /**
     * Drives the carfax at a certain speed
     * @param speed The speed of both motors
     */
    public void driveWithEncoder(int speed) {
        RIGHT.driveWithEncoder(speed);
        LEFT.driveWithEncoder(speed);
    }

    /**
     * Sends a certain voltage to the carfax
     * @param rightPower The voltage to send to the right motor
     * @param leftPower The voltage to send to the left motor
     */
    public void driveWithoutEncoder(int rightPower, int leftPower) {
        RIGHT.driveWithoutEncoder(rightPower);
        LEFT.driveWithoutEncoder(leftPower);
    }

    /**
     * Sends a certain voltage to the holonomic drivetrain
     * @param power The voltage to send
     */
    public void driveWithoutEncoder(int power) {
        RIGHT.driveWithoutEncoder(power);
        LEFT.driveWithoutEncoder(power);
    }

    /**
     * Brings the carfax to a stop and resets it
     */
    public void stop() {
        brake();
        reset();
    }

    /**
     * Brings the carfax to a stop
     */
    public void brake() {
        RIGHT.brake();
        LEFT.brake();
    }

    /**
     * Resets the carfax
     */
    public void reset() {
        RIGHT.reset();
        LEFT.reset();
    }

    public StandardMotor getRightTop() {
        return RIGHT;
    }
    public StandardMotor getLeftTop() {
        return LEFT;
    }
    public StandardMotor getLeftBottom() {
        return null;
    }
    public StandardMotor getRightBottom() {
        return null;
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
