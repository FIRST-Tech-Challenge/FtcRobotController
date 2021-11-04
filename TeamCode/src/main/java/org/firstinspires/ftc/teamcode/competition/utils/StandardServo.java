package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;

public class StandardServo {

    private final Servo SERVO;

    /**
     * Creates a new Servo
     * @param hardware The HardwareMap to get the physical servo from
     * @param name The servo's name in the configuration
     */
    public StandardServo(HardwareMap hardware, String name) {
        SERVO = hardware.get(Servo.class, name);
        SERVO.resetDeviceConfigurationForOpMode();
        SERVO.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Sets the position of the servo, between -180 and 180
     * @param position The position to set
     * @throws ArithmeticException The exception thrown when the position is not between -180 and 180
     */
    public void setPosition(int position) throws ArithmeticException {
        if(position >= -180 && position <= 180) {
            SERVO.setPosition(position / 180.0);
        }else{
            throw new ArithmeticException("Position must be between -100 and 100!");
        }
    }

    public void setDirection(Servo.Direction direction) {
        SERVO.setDirection(direction);
    }

    public Servo getInternalServo() {
        return SERVO;
    }

    public ServoController getController() {
        return SERVO.getController();
    }

}
