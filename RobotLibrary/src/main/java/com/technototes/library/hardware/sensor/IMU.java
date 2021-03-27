package com.technototes.library.hardware.sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.technototes.library.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/** Class for imus
 *
 */
public class IMU {

    private BNO055IMU.Parameters parameters;
    /** The imu
     *
     */
    public BNO055IMU device;

    /** Make an imu
     *
     * @param device The imu device
     */
    public IMU(BNO055IMU device) {
        this.device = device;
        parameters = new BNO055IMU.Parameters();
        degrees();
        this.device.initialize(parameters);

    }

    /** Make an imu
     *
     * @param deviceName The device name in hardware map
     */
    public IMU(String deviceName) {
        device = HardwareDevice.hardwareMap.get(BNO055IMU.class, deviceName);
        parameters = new BNO055IMU.Parameters();
        degrees();
        device.initialize(parameters);

    }

    /** Set angle format to degrees
     *
     * @return this
     */
    public IMU degrees(){
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        return this;
    }
    /** Set angle format to radians
     *
     * @return this
     */
    public IMU radians(){
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        return this;
    }

    public IMU initialize(){
        device.initialize(parameters);
        return this;
    }

    public double getSensorValue() {
        return gyroHeading();
    }

    /** Get gyro heading
     *
     * @return The gyro heading
     */
    public double gyroHeading() {
        Orientation angles1 =
                device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        device.getParameters().angleUnit == BNO055IMU.AngleUnit.DEGREES? AngleUnit.DEGREES : AngleUnit.RADIANS);
        return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
    }
}
