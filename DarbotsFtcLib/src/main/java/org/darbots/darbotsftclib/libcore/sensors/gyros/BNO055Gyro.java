package org.darbots.darbotsftclib.libcore.sensors.gyros;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BNO055Gyro implements RobotGyro, RobotNonBlockingDevice {
    private BNO055IMU m_BNO055Gyro;
    private Orientation m_angles;
    public BNO055Gyro(HardwareMap hardwareMap, String GyroName){
        m_BNO055Gyro = hardwareMap.get(BNO055IMU.class,GyroName);
        __initGyro();
    }
    protected void __initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        m_BNO055Gyro.initialize(parameters);
        this.updateData();
    }


    protected void updateData() {
        m_angles = m_BNO055Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public float getX(){
        return XYPlaneCalculations.normalizeDeg(m_angles.thirdAngle);
    }

    public float getY(){
        return XYPlaneCalculations.normalizeDeg(m_angles.secondAngle);
    }

    public float getZ(){
        return XYPlaneCalculations.normalizeDeg(m_angles.firstAngle);
    }

    @Override
    public float getHeading() {
        return this.getZ();
    }

    @Override
    public HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return HeadingRotationPositiveOrientation.CounterClockwise;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {
        this.updateData();
    }

    @Override
    public void waitUntilFinish() {
        return;
    }
}
