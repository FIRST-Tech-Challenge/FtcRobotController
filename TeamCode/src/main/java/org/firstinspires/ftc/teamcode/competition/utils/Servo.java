package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Simple implementation of Qualcomm's Servo interface allowing for creation of a servo with a constructor.
 * @author Thomas Ricci
 */
public class Servo implements com.qualcomm.robotcore.hardware.Servo {

    private final com.qualcomm.robotcore.hardware.Servo INTERNAL_SERVO;

    /**
     * Creates a servo
     * @param hardware The hardware map the servo's associated with
     * @param name The name of the servo to represent
     */
    public Servo(HardwareMap hardware, String name) {
        INTERNAL_SERVO = hardware.get(com.qualcomm.robotcore.hardware.Servo.class, name);
    }

    @Override
    public ServoController getController() {
        return INTERNAL_SERVO.getController();
    }

    @Override
    public int getPortNumber() {
        return INTERNAL_SERVO.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        INTERNAL_SERVO.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return INTERNAL_SERVO.getDirection();
    }

    @Override
    public void setPosition(double position) {
        INTERNAL_SERVO.setPosition(position);
    }

    @Override
    public double getPosition() {
        return INTERNAL_SERVO.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        INTERNAL_SERVO.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return INTERNAL_SERVO.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return INTERNAL_SERVO.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return INTERNAL_SERVO.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return INTERNAL_SERVO.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        INTERNAL_SERVO.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        INTERNAL_SERVO.close();
    }
}
