package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;

public class RFDualServo implements Servo {
    /*fanmcy init
     * two servos
     * position seperation 1.0
     * if willy wills it, create a telly that has three buttons: flip first servo to 1.0/0.0, flip second servo to 1.0/0.0, flip both servos to 1.0/0.0*/

    private Servo dualServo1;
    private Servo dualServo2;

    private double lasttime = 0;

    double servolimit = 0;
    boolean flipped = false;

    Servo.Direction servoDirection1;

    private String rfDualServoName;

    private ArrayList<String> inputlogs = new ArrayList<>();

    public RFDualServo(String deviceName1, String deviceName2, double limit) {

        dualServo1 = op.hardwareMap.servo.get(deviceName1);
        dualServo2 = op.hardwareMap.servo.get(deviceName2);

        servolimit = limit;

        rfDualServoName = deviceName1;

        logger.createFile("/DualServoLogs/RFDualServo", "sigh");
    }

    public RFDualServo(Servo.Direction servoDirection, String deviceName1, String deviceName2, double limit) {

        dualServo1 = op.hardwareMap.servo.get(deviceName1);
        dualServo2 = op.hardwareMap.servo.get(deviceName2);

        dualServo1.setDirection(servoDirection);

        if (servoDirection == REVERSE) {
            dualServo2.setDirection(FORWARD);
        }
        else {
            dualServo2.setDirection(REVERSE);
        }

        servolimit = limit;

        rfDualServoName = deviceName1;
    }

    public void flipServosMax (){
        if (op.getRuntime() - lasttime > 0.2) {
            if (!flipped) {
//                dualServo1.setPosition(servolimit);
//                dualServo2.setPosition(0);
                setPositions(servolimit);
                flipped = true;
            } else {
//                dualServo1.setPosition(0);
//                dualServo2.setPosition(servolimit);
                setPositions(0);
                flipped = false;
            }
            lasttime = op.getRuntime();
        }
    }

    public void flipServosInterval (double servo1lowerpos, double servo1upperpos){
        if (op.getRuntime() - lasttime > 0.2) {
            if (!flipped) {
                setPositions(servo1upperpos);
                flipped = true;
            } else {
                setPositions(servo1lowerpos);
                flipped = false;
            }
            lasttime = op.getRuntime();
        }
    }

    public void setPositions(double position) {
        if (position <= servolimit) {
            if (dualServo1.getPosition() != position) {
                inputlogs.add(rfDualServoName);
                inputlogs.add("setPositions()");
                inputlogs.add("Setting Positions: servo 1: " + position + ", servo 2: " + (servolimit - position));
                logger.log("/RobotLogs/GeneralRobot", inputlogs);
                inputlogs.clear();

//                logger.log("/DualServoLogs/RFDualServo", " Setting Positions: " + position + ", " + servolimit + position);
//                logger.log("/RobotLogs/GeneralRobot", rfDualServoName + "\nsetPositions():\nSetting Positions:\nservo 1: " + position + "\nservo 2: " + (servolimit - position));

            }
            dualServo1.setPosition(position);
            dualServo2.setPosition(servolimit - position);
            lasttime = op.getRuntime();
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
        logger.logMessage("RFServoLog", "error: function body empty, check rf dual servo code");
    }

    public void scaleRange(double min, double max) {
        logger.logMessage("RFServoLog", "error: function body empty, check rf dual servo code");
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
        logger.logMessage("RFServoLog", "error: function body empty, check rf dual servo code");
    }

    @Override
    public void close() {
        logger.logMessage("RFServoLog", "error: function body empty, check rf dual servo code");
    }
}
