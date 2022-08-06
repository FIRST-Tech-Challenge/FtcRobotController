package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RFCRServo implements CRServo {
    //all servo regular stuff

    private CRServo RFCRServo;

    LinearOpMode op;

    Servo.Direction servodirection;
    String devicename;

    public RFCRServo (Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        op = opMode;
        servodirection = direction;
        devicename = deviceName;
        RFCRServo = opMode.hardwareMap.get(CRServo.class, deviceName);
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
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double v) {
        RFCRServo.setPower(v);
    }

    @Override
    public double getPower() {
        return 0;
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
