package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.Robot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFDualServo implements Servo {
    /*fanmcy init
     * two servos
     * position seperation 1.0
     * if willy wills it, create a telly that has three buttons: flip first servo to 1.0/0.0, flip second servo to 1.0/0.0, flip both servos to 1.0/0.0*/

    private Servo dualServo1;
    private Servo dualServo2;

    double MIN_POSITION = 0.0;
    double MAX_POSITION = 1.0;
    double servolimit = 0;
    boolean flipped = false;

    enum Direction { FORWARD, REVERSE }

    Servo.Direction servoDirection1;

    LinearOpMode op;

    public RFDualServo(LinearOpMode opMode, String deviceName1, String deviceName2, double limit) {
        op = opMode;

        dualServo1 = opMode.hardwareMap.servo.get(deviceName1);
        dualServo2 = opMode.hardwareMap.servo.get(deviceName2);

        servolimit = limit;
    }

    public RFDualServo(Servo.Direction servoDirection, LinearOpMode opMode, String deviceName1, String deviceName2, double limit) {
        op = opMode;

        dualServo1 = opMode.hardwareMap.servo.get(deviceName1);
        dualServo2 = opMode.hardwareMap.servo.get(deviceName2);

        dualServo1.setDirection(servoDirection);

        if (servoDirection == REVERSE) {
            dualServo2.setDirection(FORWARD);
        }
        else {
            dualServo2.setDirection(REVERSE);
        }

        servolimit = limit;
    }

    public void flipServos (){
        if (!flipped) {
            dualServo1.setPosition(servolimit);
            dualServo2.setPosition(0);
            flipped = true;
        }
        else {
            dualServo1.setPosition(0);
            dualServo2.setPosition(servolimit);
            flipped = false;
        }

    }

    public void setPositions(double position) {
        if (position < servolimit) {
            dualServo1.setPosition(position);
            dualServo2.setPosition(servolimit - position);
        }
    }


    @Override
    public double getPosition() {
        return dualServo1.getPosition();
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
        dualServo1.setDirection(direction);

        dualServo2.setDirection(REVERSE);

        if (direction == REVERSE) {
            dualServo2.setDirection(FORWARD);
        }
    }

    public Servo.Direction getDirection() {
        return servoDirection1;
    }

    @Override
    public void setPosition(double position) {

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
