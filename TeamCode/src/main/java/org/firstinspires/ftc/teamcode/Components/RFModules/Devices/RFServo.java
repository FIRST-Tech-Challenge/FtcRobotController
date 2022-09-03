package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.BasicRobot.logger;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RFServo implements Servo {
    //all servo regular stuff

    private final Servo RFServo;

    String devicename;

    private double lasttime = 0;



    public RFServo (Servo.Direction direction, String deviceName) {
        devicename = deviceName;
        RFServo = op.hardwareMap.get(Servo.class, deviceName);
        RFServo.setDirection(direction);

    }

    public void setPosition(double position) {
        if (op.getRuntime() - lasttime > 0.2) {
            RFServo.setPosition(position);
            logger.log("RFServoLog", "Setting Position:" + position);
            lasttime = op.getRuntime();
        }
    }

    public double getPosition() {
        logger.log("RFServoLog", "Current Position:" + RFServo.getPosition());
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
        return devicename;
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
