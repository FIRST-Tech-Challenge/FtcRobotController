package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFDualServo implements Servo {
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

    public RFDualServo(Servo.Direction servoDirection, LinearOpMode opMode) {
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

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
