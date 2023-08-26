package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
// NOT TESTED YET - 7/28/2023, PUSHING AFTER SOME MORE CLEANUP
public class RFCRServo implements CRServo {
    private final CRServo crServo;
    private final String deviceName;

    /**CRServo with specified direction
     *
     * @param c_direction DcMotorSimple.Direction
     * @param p_deviceName String for hwMap
     */
    public RFCRServo (String p_deviceName, DcMotorSimple.Direction c_direction) {
        deviceName = p_deviceName;
        crServo = op.hardwareMap.get(CRServo.class, p_deviceName);
        crServo.setDirection(c_direction);
    }

    /**CRServo using default direction
     *
     * @param p_deviceName String for hwMap
     */
    public RFCRServo (String p_deviceName) {
        deviceName = p_deviceName;
        crServo = op.hardwareMap.get(CRServo.class, p_deviceName);
    }

    // movement methods

    /**
     * set power to 1, turns clockwise
     */
    public void spinClockwise() {
        crServo.setPower(1.0);
    }

    /**
     * set power to -1, turns counter clockwise
     */
    public void spinCounterClockwise() {
        crServo.setPower(-1.0);
    }

    /**
     *  set power to 0, stops turning
     */
    public void stopSpinning() {
        crServo.setPower(0);
    }

    /**
     * these methods override the methods in the crservo class,
     * which rfcrservo implements,
     * in order to compile
     */
    @Override
    public ServoController getController() {
        return crServo.getController();
    }

    @Override
    public int getPortNumber() {
        return crServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {crServo.setDirection(direction);}

    @Override
    public Direction getDirection() {
        return crServo.getDirection();
    }

    @Override
    public void setPower(double v) {
        crServo.setPower(v);
    }

    @Override
    public double getPower() {
        return crServo.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return crServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return crServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return crServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return crServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {crServo.resetDeviceConfigurationForOpMode();}

    @Override
    public void close() {
        crServo.close();
    }
}
