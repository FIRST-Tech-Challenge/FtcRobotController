package org.firstinspires.ftc.teamcode.competition.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tank {

    private final Telemetry TELEMETRY;
    private final Motor RIGHT_TOP, RIGHT_BOTTOM, LEFT_TOP, LEFT_BOTTOM;

    /**
     * Creates a new tank drivetrain.
     * @param telemetry The telemetry to log to
     * @param rightTop The right top motor of the tank
     * @param rightBottom The right bottom motor of the tank
     * @param leftTop The left top motor of the tank
     * @param leftBottom The left bottom motor of the tank
     */
    public Tank(Telemetry telemetry, Motor rightTop, Motor rightBottom, Motor leftTop, Motor leftBottom) {
        RIGHT_TOP = rightTop;
        RIGHT_BOTTOM = rightBottom;
        LEFT_TOP = leftTop;
        LEFT_BOTTOM = leftBottom;
        TELEMETRY = telemetry;
        TELEMETRY.addData("Tank Ready", "Motors added and tank ready to drive.");
    }

    /**
     * Drives the tank a distance
     * @param rightDistance The distance of the right two motors
     * @param leftDistance The distance of the left two motors
     * @param speed The speed of all four motors
     */
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
    public void driveWithoutEncoder(int power) {
        RIGHT_TOP.driveWithoutEncoder(power);
        RIGHT_BOTTOM.driveWithoutEncoder(power);
        LEFT_TOP.driveWithoutEncoder(power);
        LEFT_BOTTOM.driveWithoutEncoder(power);
    }

    /**
     * Brings the tank to a stop and resets it
     */
    public void stop() {
        brake();
        reset();
    }

    /**
     * Brings the tank to a stop
     */
    public void brake() {
        RIGHT_TOP.brake();
        RIGHT_BOTTOM.brake();
        LEFT_TOP.brake();
        LEFT_BOTTOM.brake();
    }

    /**
     * Resets the tank
     */
    public void reset() {
        RIGHT_TOP.reset();
        RIGHT_BOTTOM.reset();
        LEFT_TOP.reset();
        LEFT_BOTTOM.reset();
    }

    public Telemetry getTelemetry() {
        return TELEMETRY;
    }

    public Motor getRightTop() {
        return RIGHT_TOP;
    }

    public Motor getRightBottom() {
        return RIGHT_BOTTOM;
    }

    public Motor getLeftTop() {
        return LEFT_TOP;
    }

    public Motor getLeftBottom() {
        return LEFT_BOTTOM;
    }

}
