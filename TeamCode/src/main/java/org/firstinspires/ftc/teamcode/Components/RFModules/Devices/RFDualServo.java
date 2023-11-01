package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger.df;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 * William
 */
public class RFDualServo implements Servo {
    /**
     * init
     * two servos
     * last time servo was flipped, set to -1 because it will be changed later
     * 2 decimal places, used in logger
     * create class variables
     */

    private Servo dualServo1;
    private Servo dualServo2;

    private double lastTime = -1;

    private double servoLimit;
    private boolean flipped = false;

    private final double FLIP_TIME_SEC = 0.2;

    private Servo.Direction servoDirection1;

    private String rfDualServoName;

    private ArrayList<String> inputlogs = new ArrayList<>();

    /**
     * Dual Servo when not given a specific direction
     * @param p_dualServoName name of the set of servos for logging
     * @param p_deviceName1 name of the first servo in the hardware map
     * @param p_deviceName2 name of the second servo in the hardware map
     * @param limit upper limit of the servo's motion
     * Logs which state(s)' values have been changed and to what.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public RFDualServo(String p_dualServoName, String p_deviceName1, String p_deviceName2, double limit) {
        this(p_dualServoName, FORWARD, p_deviceName1,p_deviceName2,limit);
    }

    /**
     * Dual Servo when not given a specific direction
     * @param p_dualServoName name of the set of servos for logging
     * @param p_servoDirection direction of the first servo
     * @param p_deviceName1 name of the first servo in the hardware map
     * @param p_deviceName2 name of the second servo in the hardware map
     * @param limit upper limit of the servo's motion
     * Logs which state(s)' values have been changed and to what.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public RFDualServo(String p_dualServoName, Servo.Direction p_servoDirection, String p_deviceName1, String p_deviceName2, double limit) {

        dualServo1 = op.hardwareMap.servo.get(p_deviceName1);
        dualServo2 = op.hardwareMap.servo.get(p_deviceName2);

        dualServo1.setDirection(p_servoDirection);

        if (p_servoDirection == REVERSE) {
            dualServo2.setDirection(FORWARD);
        } else {
            dualServo2.setDirection(REVERSE);
        }

        servoLimit = limit;

        rfDualServoName = p_dualServoName;
    }

    /**
     * Sets last time the servo was flipped.
     * @param p_lastTime
     * Logs what lastTime has been changed to.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public void setLasttime(double p_lastTime) {
        lastTime = p_lastTime;
    }

    /**
     * Flips the servos to the upper or lower limit.
     * Logs which limit the servos have been flipped to.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public void flipServosMax() {
        if (op.getRuntime() - lastTime > FLIP_TIME_SEC) {
            if (!flipped) {
                setPositions(servoLimit);
                flipped = true;
            } else {
                setPositions(0);
                flipped = false;
            }
            lastTime = op.getRuntime();
        }
    }

    /**
     * Sets last time the servo was flipped.
     * @param p_servo1lowerpos lower limit of the first servo
     * @param p_servo1upperpos upper limit of the first servo
     * Logs what limit the servos have been flipped to.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public void flipServosInterval(double p_servo1lowerpos, double p_servo1upperpos) {
        if (op.getRuntime() - lastTime > FLIP_TIME_SEC) {
            if (!flipped) {
                setPositions(p_servo1upperpos);
                flipped = true;
            } else {
                setPositions(p_servo1lowerpos);
                flipped = false;
            }
        }
    }

    /**
     * Sets positions of the servos.
     * @param p_position target position
     * Logs what positions the servos have been flipped to.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public void setPositions(double p_position) {
        if (p_position <= servoLimit) {
            if (dualServo1.getPosition() != p_position) {
                dualServo1.setPosition(p_position);
                dualServo2.setPosition(servoLimit - p_position);
                logger.log("/DualServoLogs/RFDualServo", rfDualServoName +
                        ",setPositions(),Setting Positions: " + df.format(p_position) + " " +
                        df.format(servoLimit - p_position), true);
                lastTime = op.getRuntime();
            }
        }
    }

    /**
     * Returns the last time the servos were flipped
     * @return lastTime last time the servo were flipped
     * Logs value of lastTime.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public double getLastTime() {
        return lastTime;
    }

    /**
     * Removes all power from the servos, used for cone flipper during Power Play season.
     * Logs that servos have been disabled.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public void disableServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        rfServoController1.pwmDisable();
        rfServoController2.pwmDisable();
    }

    /**
     * Adds power to the servos, used for cone flipper during Power Play season.
     * Logs that servos have been disabled.
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public void enableServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        rfServoController1.pwmEnable();
        rfServoController2.pwmEnable();
    }

    /**
     * Returns if the servos are enabled or not.
     * @return status of servo
     * Logs if servos are enabled or disabled
     * Logs to RFDualServo & general logs.
     * Logs to least fine level.
     * Does not update any state machines.
     */
    public boolean abledServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        return rfServoController1.getPwmStatus() == ServoController.PwmStatus.ENABLED;
    }

    /**
     * Overrides, needed so implemented class does not cause compilation error.
     */
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
    public void setDirection(Servo.Direction direction) {
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
