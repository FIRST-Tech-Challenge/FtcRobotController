package org.firstinspires.ftc.dragonswpilib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.interfaces.Gyro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FTC_Gyro implements Gyro {

    private final HardwareMap mHardwareMap;
    private final BNO055IMU mImu;
    private double mCalibration = 0;

    public FTC_Gyro(HardwareMap hardwareMap) {
        mHardwareMap = hardwareMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        mImu = mHardwareMap.get(BNO055IMU.class, "imu");
        mImu.initialize(parameters);
    }

    @Override
    public void calibrate() {
    }

    @Override
    public void reset() {
        mCalibration = mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    @Override
    public double getAngle() {
        return mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - mCalibration;
    }

    @Override
    public double getRate() {
        return mImu.getAcceleration().zAccel;
    }

}