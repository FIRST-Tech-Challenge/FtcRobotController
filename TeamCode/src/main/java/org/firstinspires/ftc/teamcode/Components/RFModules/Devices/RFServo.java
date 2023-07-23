package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger.df;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RFServo implements Servo {
    /* Basic Servo device class with reusable code */

    private final Servo rfServo;

    private double lastTime = -100;

    double SERVO_LIMIT;

    boolean flipped = false;

    private final String rfServoName;

    /* Constructor with name, direction, and limit */

    public RFServo (String deviceName, Servo.Direction direction, double limit) {
        rfServo = op.hardwareMap.get(Servo.class, deviceName);
        rfServoName = deviceName;
        rfServo.setDirection(direction);

        logger.createFile("/ServoLogs/RFServo", "Runtime    Component               " +
                "Function               Action");

        SERVO_LIMIT = limit;
    }

    /* Constructor with name and limit */

    public RFServo (String deviceName, double limit) {
        rfServo = op.hardwareMap.get(Servo.class, deviceName);
        rfServoName = deviceName;

        logger.createFile("/ServoLogs/RFServo", "Runtime    Component               " +
                "Function               Action");

        SERVO_LIMIT = limit;
    }

    /* Updating the last time the servo flipped */

    public void setFlipTime(double p_flipTime){
        lastTime=p_flipTime;
    }

    /* Setting position of the servo */

    public void setPosition(double position) {
        if (time - lastTime > 0.2) {
                logger.log("/ServoLogs/RFServo", rfServoName + ",setPosition(),Setting Position: "
                        + df.format(position), true);
            rfServo.setPosition(position);
            lastTime = time;
        }
    }

    /* Flipping the servo between two positions inside the max range */

    public boolean flipServoInterval(double lowerPos, double upperPos) {
        if(time - lastTime > 0.2) {
            if (flipped) {
                rfServo.setPosition(lowerPos);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoInterval(),Setting Position: "
                        + df.format(lowerPos), true);
                flipped = false;
            } else {
                rfServo.setPosition(upperPos);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoInterval(),Setting Position: "
                        + df.format(upperPos), true);
                flipped = true;
            }
        }
        return time - lastTime > 0.2;
    }

    /* Flipping the servo in the max range */

    public void flipServoMax() {
        if (time - lastTime > 0.2) {
            if (flipped) {
                rfServo.setPosition(0);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoMax(),Setting Position: "
                        + df.format(0), true);
                flipped = false;
            } else {
                rfServo.setPosition(SERVO_LIMIT);
                logger.log("/ServoLogs/RFServo", rfServoName + ",flipServoMax(),Setting Position: "
                        + df.format(SERVO_LIMIT), true);
                flipped = true;
            }
            lastTime = time;
        }
    }

    /* Returns current position of the servo */

    public double getPosition() {
        return rfServo.getPosition();
    }

    /* Returns last time the servo was flipped */

    public double getLastTime() {
        return lastTime;
    }

    /* Manual update of last time servo was flipped */

    public void setLastTime(double p_lastTime){
        lastTime = p_lastTime;
    }

    /* Overridden functions from implemented Servo class */

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
