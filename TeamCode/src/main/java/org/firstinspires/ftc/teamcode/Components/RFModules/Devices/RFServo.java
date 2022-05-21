package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RFServo implements RFServoInterface {
    //all servo regular stuff

    private Servo RFServo;

    LinearOpMode op;

    Servo.Direction servodirection;
    String devicename;

    public RFServo (double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        op = opMode;
        servodirection = direction;
        devicename = deviceName;
        RFServo = opMode.hardwareMap.get(Servo.class, deviceName);
    }

    public void setPosition(double position) {
        RFServo.setPosition(position);
    }

    public double getPosition() {
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
        return servodirection;
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
