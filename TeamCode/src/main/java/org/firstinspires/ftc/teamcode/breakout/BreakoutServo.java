package org.firstinspires.ftc.teamcode.breakout;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is used to simplify the usage of Servos in the code.
 * Please use this class instead of the Servo class.
 **/
public class BreakoutServo {

    //Enum for direction
    enum Direction {
        SERVO_F(Servo.Direction.FORWARD),
        SERVO_R(Servo.Direction.REVERSE);

        private Servo.Direction servoDirection;

        Direction(Servo.Direction servoDirection) {
            this.servoDirection = servoDirection;
        }
    }

    //Define servo as a Servo object
    private Servo servo;

    //Retrieve servo var
    public Servo get() {
        return servo;
    }

    //Set hardware map data of servo
    public void set(Servo servo) {
        this.servo = servo;
    }

    //Set the direction the servo rotates in (FORWARD or REVERSE)
    public void setDirection(Direction direction) {
        servo.setDirection(direction.servoDirection);
    }

    //Set the position the servo is in (0-1)
    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() { return this.servo.getPosition(); }

}
