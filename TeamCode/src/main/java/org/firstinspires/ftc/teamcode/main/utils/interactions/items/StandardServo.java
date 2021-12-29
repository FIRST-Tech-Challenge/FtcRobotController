package org.firstinspires.ftc.teamcode.main.utils.interactions.items;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;

public class StandardServo extends InteractionItem {

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
     * Sets the position of the servo, between 0 and 100
     * @param position The position to set
     * @throws ArithmeticException The exception thrown when the position is not between -100 and 100
     */
    public void setPosition(int position) throws ArithmeticException {
        if(position >= 0 && position <= 100) {
            SERVO.setPosition(position / 100.0);
        }else{
            throw new ArithmeticException("Position must be between - and 100!");
        }
    }

    public void stop() {}

    /**
     * Gets the degree of the servo's set position. Note that this does not return the physical degree of rotation of the servo, only the position the servo is commanded to be in, as no API returning the true position is exposed by the servo
     * @return The position of the servo between 0 and 100
     */
    public int getPosition() {
        return (int) SERVO.getPosition() * 100;
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

    @Override
    public boolean isInputDevice() {
        return true;
    }

    @Override
    public boolean isOutputDevice() {
        return false;
    }
}
