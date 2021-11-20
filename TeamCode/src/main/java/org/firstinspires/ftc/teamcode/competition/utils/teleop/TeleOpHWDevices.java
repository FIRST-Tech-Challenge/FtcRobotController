package org.firstinspires.ftc.teamcode.competition.utils.teleop;

public class TeleOpHWDevices {

    private final boolean SPINNER_MOTOR, SPINNER_SERVOS, LIFT_MOTORS, LIFT_SERVO, LIFT_DROPPER, LIFT_SENSOR, DUCK_MOTOR;

    /**
     * Makes a new object containing the hardware devices a manager is allowed to use
     * @param spinnerMotor Whether the manager can use the robot's spinner
     * @param spinnerServos Whether the manager can use the robot's intake plate elevator
     * @param liftMotors Whether the manager can use the robot's lift
     * @param liftServo Whether the manager can use the robot's hand's trapper
     * @param liftDropper Whether the manager can use the robot's hand's turner
     * @param liftSensor Whether the manager can use the robot's hand's turner's distance sensor
     * @param duckMotor Whether the manager can use the robot's duck dropper spinner
     */
    public TeleOpHWDevices(boolean spinnerMotor, boolean spinnerServos, boolean liftMotors, boolean liftServo, boolean liftDropper, boolean liftSensor, boolean duckMotor) {
        SPINNER_MOTOR = spinnerMotor;
        SPINNER_SERVOS = spinnerServos;
        LIFT_MOTORS = liftMotors;
        LIFT_SERVO = liftServo;
        LIFT_DROPPER = liftDropper;
        LIFT_SENSOR = liftSensor;
        DUCK_MOTOR = duckMotor;
    }

    public boolean isSpinnerMotorAllowed() {
        return SPINNER_MOTOR;
    }

    public boolean isSpinnerServosAllowed() {
        return SPINNER_SERVOS;
    }

    public boolean isLiftMotorsAllowed() {
        return LIFT_MOTORS;
    }

    public boolean isLiftDropperAllowed() {
        return LIFT_DROPPER;
    }

    public boolean isLiftServoAllowed() {
        return LIFT_SERVO;
    }

    public boolean isLiftSensorAllowed() {
        return LIFT_SENSOR;
    }

    public boolean isDuckMotorAllowed() {
        return DUCK_MOTOR;
    }

}
