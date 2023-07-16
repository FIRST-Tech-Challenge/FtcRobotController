package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RFCRServo implements CRServo {
    // servo init
    private CRServo RFCRServo;

    Servo.Direction servoDirection;
    String deviceName;
    //crservo with specified direction
    public RFCRServo (Servo.Direction direction, String c_deviceName) {
        servoDirection = direction;
        deviceName = c_deviceName;
        RFCRServo = op.hardwareMap.get(CRServo.class, c_deviceName);
    }
    //crservo using default direction
    public RFCRServo (String c_deviceName) {
        deviceName = c_deviceName;
        RFCRServo = op.hardwareMap.get(CRServo.class, c_deviceName);
    }
    //movement methods
    public void spinClockwise() {
        RFCRServo.setPower(1.0);
    }

    public void spinCounterClockwise() {
        RFCRServo.setPower(-1.0);
    }

    public void stopSpinning() {
        RFCRServo.setPower(0);
    }

    //these methods override the methods in the crservo class, which rfcrservo implements, in order to compile
    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {}

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
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {

    }
}
