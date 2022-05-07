package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public abstract class RFDualServo implements RFServoInterface {
    /*fanmcy init
     * two servos
     * position seperation 1.0
     * if willy wills it, create a telly that has three buttons: flip first servo to 1.0/0.0, flip second servo to 1.0/0.0, flip both servos to 1.0/0.0*/

    private RFServo rfServo;
    private RFServo rfServo2;

    double MIN_POSITION = 0.0;
    double MAX_POSITION = 1.0;

    enum Direction { FORWARD, REVERSE }

    Servo.Direction servoDirection1;

    LinearOpMode op;

    public RFDualServo(String servoName, String servoName2, Servo.Direction servoDirection, LinearOpMode opMode) {
        op = opMode;

        servoDirection1 = servoDirection;

        Servo.Direction servoDirection2 = REVERSE;

        if (servoDirection == REVERSE) {
            servoDirection2 = FORWARD;
        }

        rfServo = new RFServo(1.0, servoDirection, "RFServo", op);
        rfServo2 = new RFServo(1.0, servoDirection2, "RFServo2", op);

        rfServo.setDirection(servoDirection);

    }


    @Override
    public void setPosition(double position) {
        rfServo.setPosition(position);
        rfServo2.setPosition(position);
    }

    @Override
    public double getPosition() {
        return rfServo.getPosition();
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection (Servo.Direction direction) {
        rfServo.setDirection(direction);

        rfServo2.setDirection(REVERSE);

        if (direction == REVERSE) {
            rfServo2.setDirection(FORWARD);
        }
    }

    public Servo.Direction getDirection() {
        return servoDirection1;
    }

    public void scaleRange(double min, double max) {

    }
}
