package org.darbots.darbotsftclib.libcore.sensors.gyros;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BNO055GyroMethod implements GyroMethod {
    private BNO055IMU m_BNO055Gyro;
    private GyroContainer m_Container;

    public BNO055GyroMethod(HardwareMap hardwareMap, String GyroName){
        m_BNO055Gyro = hardwareMap.get(BNO055IMU.class,GyroName);
    }

    protected void __initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        m_BNO055Gyro.initialize(parameters);
    }

    @Override
    public void initGyro() {
        this.__initGyro();
    }

    @Override
    public void updateData() {
        Orientation angles = m_BNO055Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.m_Container.setHeading(XYPlaneCalculations.normalizeDeg(angles.firstAngle));
    }

    @Override
    public void setGyroContainer(GyroContainer container) {
        m_Container = container;
    }

    @Override
    public RobotGyro.HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return RobotGyro.HeadingRotationPositiveOrientation.CounterClockwise;
    }
}
