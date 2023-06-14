package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import java.text.DecimalFormat;
import java.util.ArrayList;

public class RFServo implements Servo {
    //all servo regular stuff

    private final Servo rfServo;

    private double lasttime = -100;

    double servolimit = 0;

    boolean flipped = false;

    private String rfServoName;

    private ArrayList<String> inputlogs = new ArrayList<>();

    private static final DecimalFormat df = new DecimalFormat("0.00");

    public RFServo (String deviceName, Servo.Direction direction, double limit) {
        rfServo = op.hardwareMap.get(Servo.class, deviceName);
        rfServoName = deviceName;
        rfServo.setDirection(direction);

        logger.createFile("/ServoLogs/RFServo", "Runtime    Component               " +
                "Function               Action");

        servolimit = limit;
    }

    public RFServo (String deviceName, double limit) {
        rfServo = op.hardwareMap.get(Servo.class, deviceName);
        rfServoName = deviceName;

        logger.createFile("/ServoLogs/RFServo", "Runtime    Component               " +
                "Function               Action");

        servolimit = limit;
    }
    public void setFlipTime(double p_flipTime){
        lasttime=p_flipTime;
    }

    public void setPosition(double position) {
        if (time - lasttime > 0.2) {
//            if (rfServo.getPosition() != position) {
//                inputlogs.clear();
//                inputlogs.add(rfServoName);
//                inputlogs.add("setPosition()");
//                inputlogs.add("Setting Position: " + rfServo.getPosition());
                logger.log("/ServoLogs/RFServo", rfServoName + ",setPosition(),Setting Position: "
                        + df.format(position), true);
//                inputlogs.clear();

//                logger.logRegulated("/ServoLogs/RFServo", "Setting Position:" + position);
//                logger.logRegulated("/RobotLogs/GeneralRobot", rfServoName + "\nsetPosition():\nSetting Position:" + position);

            rfServo.setPosition(position);
            lasttime = time;
        }
    }

    public boolean flipServoInterval(double lowerpos, double upperpos) {
        if(time - lasttime > 0.2) {
            if (flipped) {
                rfServo.setPosition(lowerpos);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoInterval(),Setting Position: "
                        + df.format(lowerpos), true);
                flipped = false;
            } else {
                rfServo.setPosition(upperpos);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoInterval(),Setting Position: "
                        + df.format(upperpos), true);
                flipped = true;
            }
        }
        return time - lasttime > 0.2;
    }

    public void flipServoMax() {
        if (time - lasttime > 0.2) {
            if (flipped) {
                rfServo.setPosition(0);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoMax(),Setting Position: "
                        + df.format(0), true);
                flipped = false;
            } else {
                rfServo.setPosition(servolimit);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoMax(),Setting Position: "
                        + df.format(servolimit), true);
                flipped = true;
            }
            lasttime = time;
        }
    }

    public double getPosition() {
//        inputlogs.add(rfServoName);
//        inputlogs.add("getPosition()");
//        inputlogs.add("Getting Position: " + rfServo.getPosition());
//        logger.log("/RobotLogs/GeneralRobot", inputlogs);
//        logger.log("RFServoLog", "Current Position:" + rfServo.getPosition());
        return rfServo.getPosition();
    }

    public double getLastTime() {
        return lasttime;
    }
    public void setLastTime(double lastTime){
        lasttime = lastTime;
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
        rfServo.setDirection(direction);
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
