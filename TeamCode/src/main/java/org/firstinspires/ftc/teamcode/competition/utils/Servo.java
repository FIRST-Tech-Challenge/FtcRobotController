package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

public class Servo {

    private final com.qualcomm.robotcore.hardware.Servo SERVO;

    public enum ServoDirection {
        FORWARD,
        REVERSE
    }

    /**
     * Creates a new Servo
     * @param hardware The HardwareMap to get the physical servo from
     * @param name The servo's name in the configuration
     */
    public Servo(HardwareMap hardware, String name) {
        SERVO = hardware.get(com.qualcomm.robotcore.hardware.Servo.class, name);
        SERVO.resetDeviceConfigurationForOpMode();
        SERVO.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
    }

    /**
     * Sets the position of the servo, between -100 and 100
     * @param position The position to set
     * @throws ArithmeticException The exception thrown when the position is not between -100 and 100
     */
    public void setPosition(int position) throws ArithmeticException {
        if(position >= -100 && position <= 100) {
            SERVO.setPosition(position / 100.0);
        }else{
            throw new ArithmeticException("Position must be between -100 and 100!");
        }
    }

    public void setDirection(ServoDirection direction) {
        if(direction == ServoDirection.FORWARD) {
            SERVO.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
        }else if(direction == ServoDirection.REVERSE) {
            SERVO.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        }
    }

    public com.qualcomm.robotcore.hardware.Servo getInternalServo() {
        return SERVO;
    }

    public ServoController getController() {
        return SERVO.getController();
    }

}
