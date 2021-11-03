package org.firstinspires.ftc.teamcode.competition.utils.teleop;

public class TeleOpHWDevices {

    private final boolean SPINNER_MOTOR, SPINNER_SERVO, LIFT_MOTOR, LIFT_SERVO, LIFT_SENSOR, DUCK_MOTOR;

    /**
     * Makes a new object containing the hardware devices a manager is allowed to use
     * @param spinnerMotor Whether the manager can use the robot's spinner
     * @param spinnerServo Whether the manager can use the robot's intake plate elevator
     * @param liftMotor Whether the manager can use the robot's lift
     * @param liftServo Whether the manager can use the robot's hand's turner
     * @param liftSensor Whether the manager can use the robot's hand's turner's distance sensor
     * @param duckMotor Whether the manager can use the robot's duck dropper spinner
     */
    public TeleOpHWDevices(boolean spinnerMotor, boolean spinnerServo, boolean liftMotor, boolean liftServo, boolean liftSensor, boolean duckMotor) {
        SPINNER_MOTOR = spinnerMotor;
        SPINNER_SERVO = spinnerServo;
        LIFT_MOTOR = liftMotor;
        LIFT_SERVO = liftServo;
        LIFT_SENSOR = liftSensor;
        DUCK_MOTOR = duckMotor;
    }

    public boolean isSpinnerMotorAllowed() {
        return SPINNER_MOTOR;
    }

    public boolean isSpinnerServoAllowed() {
        return SPINNER_SERVO;
    }

    public boolean isLiftMotorAllowed() {
        return LIFT_MOTOR;
    }

    public boolean isLiftSensorAllowed() {
        return LIFT_SENSOR;
    }

    public boolean isDuckMotorAllowed() {
        return DUCK_MOTOR;
    }

}
