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
    /**init
     * two servos
     * last time servo was flipped, set to -100 because it will be changed later
     * 2 decimal places, used in logger
     * create class variables
     */

    private Servo dualServo1;
    private Servo dualServo2;

    private double lastTime = -100;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    double servoLimit = 0;
    boolean flipped = false;

    Servo.Direction servoDirection1;

    private String rfDualServoName;

    private ArrayList<String> inputlogs = new ArrayList<>();

    /**Dual Servo when not given a specific direction
     *
     * @param deviceName1
     * @param deviceName2
     * @param limit
     */
    public RFDualServo(String deviceName1, String deviceName2, double limit) {
        //hwmap
        dualServo1 = op.hardwareMap.servo.get(deviceName1);
        dualServo2 = op.hardwareMap.servo.get(deviceName2);
        //sets the servo limit to the passed in limit
        servoLimit = limit;

        rfDualServoName = deviceName1;

        logger.createFile("/DualServoLogs/RFDualServo", "Runtime    Component               " +
                "Function               Action");
    }

    /**Dual Servo w/ direction
     *
     * @param servoDirection
     * @param deviceName1
     * @param deviceName2
     * @param limit
     */
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

        servoLimit = limit;

        rfDualServoName = deviceName1;
    }

    /**sets the lastTime variable to when it was last flipped
     *
     * @param p_lastTime
     */
    public void setLasttime(double p_lastTime){
        lastTime = p_lastTime;
    }

    /**turns servos to their maximum/minimum depending on if already flipped
     *
     */
    public void flipServosMax (){
        if (op.getRuntime() - lastTime > 0.2) {
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

    /**pass in doubles for specific amount the servos should turn to
     *
     * @param servo1lowerpos
     * @param servo1upperpos
     */
    public void flipServosInterval (double servo1lowerpos, double servo1upperpos){
        if(op.getRuntime() - lastTime > 0.2) {
            if (!flipped) {
                setPositions(servo1upperpos);
                flipped = true;
            } else {
                setPositions(servo1lowerpos);
                flipped = false;
            }
        }
    }

    /**
     *
     * @param position
     */
    public void setPositions(double position) {
        if (position <= servoLimit) {
            if (dualServo1.getPosition() != position) {
                dualServo1.setPosition(position);
                dualServo2.setPosition(servoLimit - position);
                logger.log("/DualServoLogs/RFDualServo", rfDualServoName +
                        ",setPositions(),Setting Positions: " + df.format(position) + " " +
                        df.format(servoLimit - position), true);
                lastTime = op.getRuntime();
            }
        }

    }

    /**gets time of last flip
     *
     * @return
     */
    public double getLastTime() {
        return lastTime;
    }

    /**remove power from servos, used for cone flipper in PowerPlay
     *
     */
    public void disableServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        rfServoController1.pwmDisable();
        rfServoController2.pwmDisable();
    }

    /**give power back to servos
     *
     */
    public void enableServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        rfServoController1.pwmEnable();
        rfServoController2.pwmEnable();
    }

    /**returns if servos are on or not
     *
     * @return
     */
    public boolean abledServos() {
        ServoController rfServoController1 = (ServoController) dualServo1.getController();
        ServoController rfServoController2 = (ServoController) dualServo2.getController();
        if(rfServoController1.getPwmStatus()== ServoController.PwmStatus.ENABLED){
            return true;
        }else{
            return false;
        }
    }

    /**overrides are needed so implemented class does not cause compilation error
     *
     * @return
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
