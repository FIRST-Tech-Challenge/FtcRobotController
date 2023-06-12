package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.text.DecimalFormat;
import java.util.ArrayList;

public class RFDualServo implements Servo {
    /*fanmcy init
     * two servos
     * position seperation 1.0
     * if willy wills it, create a telly that has three buttons: flip first servo to 1.0/0.0, flip second servo to 1.0/0.0, flip both servos to 1.0/0.0*/

    private Servo dualServo1;
    private Servo dualServo2;

    private double lasttime = 0;

    private static final DecimalFormat df = new DecimalFormat("0.00");

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

        logger.createFile("/DualServoLogs/RFDualServo", "Runtime    Component               " +
                "Function               Action");
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
    public void setLasttime(double p_lastTime){
        lasttime = p_lastTime;
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
        if(op.getRuntime() - lasttime > 0.2) {
            if (!flipped) {
                setPositions(servo1upperpos);
                flipped = true;
            } else {
                setPositions(servo1lowerpos);
                flipped = false;
            }
        }
    }

    public void setPositions(double position) {
        if (position <= servolimit) {
            if (dualServo1.getPosition() != position) {
//                logger.log("/DualServoLogs/RFDualServo", rfDualServoName + ",setPositions()," +
//                        "Setting Positions: servo 1: " + df.format(position) + "; servo 2: " +
//                        df.format(servolimit - position), true);
//                inputlogs.add(rfDualServoName);
//                inputlogs.add("setPositions()");
//                inputlogs.add("Setting Positions: servo 1: " + position + ", servo 2: " + (servolimit - position));
//                logger.log("/RobotLogs/GeneralRobot", inputlogs);
//                inputlogs.clear();

//                logger.log("/DualServoLogs/RFDualServo", " Setting Positions: " + position + ", " + servolimit + position);
//                logger.log("/RobotLogs/GeneralRobot", rfDualServoName + "\nsetPositions():\nSetting Positions:\nservo 1: " + position + "\nservo 2: " + (servolimit - position));

//                if (time - lasttime > 0.2) {
                    dualServo1.setPosition(position);
                    dualServo2.setPosition(servolimit - position);
                    logger.log("/DualServoLogs/RFDualServo", rfDualServoName +
                            ",setPositions(),Setting Positions: " + df.format(position) + " " +
                            df.format(servolimit - position), true);
                    lasttime = op.getRuntime();
//                }
            }
        }

    }

    public double getLastTime() {
        return lasttime;
    }

    public void disableServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        rfServoController1.pwmDisable();
        rfServoController2.pwmDisable();
    }
    public void enableServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        rfServoController1.pwmEnable();
        rfServoController2.pwmEnable();
    }
    public boolean abledServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        if(rfServoController1.getPwmStatus()== ServoController.PwmStatus.ENABLED){
            return true;
        }else{
            return false;
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
        logger.log("RFDualServoLog", "error: function body empty, check rf dual servo code");
    }

    public void scaleRange(double min, double max) {
        logger.log("RFDualServoLog", "error: function body empty, check rf dual servo code");
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
        logger.log("RFServoLog", "error: function body empty, check rf dual servo code");
    }

    @Override
    public void close() {
        logger.log("RFServoLog", "error: function body empty, check rf dual servo code");
    }
}
