package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;

public class RFServo implements Servo {
    //all servo regular stuff

    private final Servo RFServo;

    private double lasttime = 0;

    double servolimit = 0;

    boolean flipped = false;

    private String rfServoName;

    private ArrayList<String> inputlogs = new ArrayList<>();

    public RFServo (Servo.Direction direction, String deviceName, double limit) {
        RFServo = op.hardwareMap.get(Servo.class, deviceName);
        rfServoName = deviceName;
        RFServo.setDirection(direction);

        logger.createFile("/ServoLogs/RFServo", "sigh");

        servolimit = limit;
    }

    public RFServo (String deviceName, double limit) {
        RFServo = op.hardwareMap.get(Servo.class, deviceName);
        rfServoName = deviceName;

        logger.createFile("/ServoLogs/RFServo", "sigh");

        servolimit = limit;
    }

    public void setPosition(double position) {
        if (op.getRuntime() - lasttime > 0.2) {
            if (RFServo.getPosition() != position) {
                inputlogs.clear();
                inputlogs.add(rfServoName);
                inputlogs.add("setPosition()");
                inputlogs.add("Setting Position: " + RFServo.getPosition());
                logger.log("/RobotLogs/GeneralRobot", inputlogs);
                inputlogs.clear();

//                logger.logRegulated("/ServoLogs/RFServo", "Setting Position:" + position);
//                logger.logRegulated("/RobotLogs/GeneralRobot", rfServoName + "\nsetPosition():\nSetting Position:" + position);
            }
            RFServo.setPosition(position);
            lasttime = op.getRuntime();
        }
    }

    public void flipServoInterval(double lowerpos, double upperpos) {
        if (op.getRuntime() - lasttime > 0.2) {
            if (flipped) {
                setPosition(lowerpos);
                flipped = false;
            }
            else {
                setPosition(upperpos);
                flipped = true;
            }
            lasttime = op.getRuntime();
        }
    }

    public void flipServoMax() {
        if (op.getRuntime() - lasttime > 0.2) {
            if (flipped) {
                setPosition(0);
                flipped = false;
            } else {
                setPosition(servolimit);
                flipped = true;
            }
            lasttime = op.getRuntime();
        }
    }

    public double getPosition() {
//        inputlogs.add(rfServoName);
//        inputlogs.add("getPosition()");
//        inputlogs.add("Getting Position: " + RFServo.getPosition());
//        logger.log("/RobotLogs/GeneralRobot", inputlogs);
//        logger.log("RFServoLog", "Current Position:" + RFServo.getPosition());
        return RFServo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    public void setDirection (Servo.Direction direction) {
        RFServo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return rfServoName;
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
