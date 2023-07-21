package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class RFCRServo implements CRServo {
    private CRServo RFCRServo;
    private Servo.Direction servoDirection;
    private String deviceName;

    /**crservo with specified direction
     *
     * @param direction
     * @param c_deviceName
     */
    public RFCRServo (Servo.Direction direction, String c_deviceName) {
        servoDirection = direction;
        deviceName = c_deviceName;
        RFCRServo = op.hardwareMap.get(CRServo.class, c_deviceName);
    }

    /**crservo using default direction
     *
     * @param c_deviceName
     */
    public RFCRServo (String c_deviceName) {
        deviceName = c_deviceName;
        RFCRServo = op.hardwareMap.get(CRServo.class, c_deviceName);
    }

    /**movement methods
     *
     */
    public void spinClockwise() {
        RFCRServo.setPower(1.0);
    }

    public void spinCounterClockwise() {
        RFCRServo.setPower(-1.0);
    }

    public void stopSpinning() {
        RFCRServo.setPower(0);
    }

    /**these methods override the methods in the crservo class, which rfcrservo implements, in order to compile
     *
     *
     */
    @Override
    public ServoController getController() {
        return RFCRServo.getController();
    }

    @Override
    public int getPortNumber() {
        return RFCRServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {RFCRServo.setDirection(direction);}

    @Override
    public Direction getDirection() {
        return RFCRServo.getDirection();
    }

    @Override
    public void setPower(double v) {
        RFCRServo.setPower(v);
    }

    @Override
    public double getPower() {
        return RFCRServo.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return RFCRServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return RFCRServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return RFCRServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return RFCRServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {RFCRServo.resetDeviceConfigurationForOpMode();}

    @Override
    public void close() {
        RFCRServo.close();
    }
}
