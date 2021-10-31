package org.firstinspires.ftc.teamcode.competition.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Car {

    private final Telemetry TELEMETRY;
    private final Motor RIGHT, LEFT;

    /**
     * Creates a new car drivetrain.
     * @param telemetry The telemetry to log to
     * @param right The right top motor of the car
     */
    public Car(Telemetry telemetry, Motor right, Motor left) {
        RIGHT = right;
        LEFT = left;
        TELEMETRY = telemetry;
        TELEMETRY.addData("Car Ready", "Motors added and car ready to drive.");
    }

    /**
     * Drives the car a distance
     * @param rightDistance The distance of the right two motors
     * @param leftDistance The distance of the left two motors
     * @param speed The speed of all four motors
     */
    public void driveDistance(int rightDistance, int leftDistance, int speed) {
        RIGHT.driveDistance(rightDistance, speed);
        LEFT.driveDistance(leftDistance, speed);
    }

    /**
     * Drives the car a distance
     * @param distance The distance of all four motors
     * @param speed The speed of all four motors
     */
    public void driveDistance(int distance, int speed) {
        RIGHT.driveDistance(distance, speed);
        LEFT.driveDistance(distance, speed);
    }

    /**
     * Drives the car at a certain speed
     * @param rightSpeed The speed of the right two motors
     * @param leftSpeed The speed of the left two motors
     */
    public void driveWithEncoder(int rightSpeed, int leftSpeed) {
        RIGHT.driveWithEncoder(rightSpeed);
        LEFT.driveWithEncoder(leftSpeed);
    }

    /**
     * Drives the car at a certain speed
     * @param speed The speed of all four motors
     */
    public void driveWithEncoder(int speed) {
        RIGHT.driveWithEncoder(speed);
        LEFT.driveWithEncoder(speed);
    }

    /**
     * Sends a certain voltage to the car
     * @param rightPower The voltage to send to the right two motors
     * @param leftPower The voltage to send to the left two motors
     */
    public void driveWithoutEncoder(int rightPower, int leftPower) {
        RIGHT.driveWithoutEncoder(rightPower);
        LEFT.driveWithoutEncoder(leftPower);
    }

    /**
     * Sends a certain voltage to the car
     * @param power The voltage to send
     */
    public void driveWithoutEncoder(int power) {
        RIGHT.driveWithoutEncoder(power);
        LEFT.driveWithoutEncoder(power);
    }

    /**
     * Brings the car to a stop and resets it
     */
    public void stop() {
        brake();
        reset();
    }

    /**
     * Brings the car to a stop
     */
    public void brake() {
        RIGHT.brake();
        LEFT.brake();
    }

    /**
     * Resets the car
     */
    public void reset() {
        RIGHT.reset();
        LEFT.reset();
    }

    public Telemetry getTelemetry() {
        return TELEMETRY;
    }

    public Motor getright() {
        return RIGHT;
    }

    public Motor getleft() {
        return LEFT;
    }
    
}
