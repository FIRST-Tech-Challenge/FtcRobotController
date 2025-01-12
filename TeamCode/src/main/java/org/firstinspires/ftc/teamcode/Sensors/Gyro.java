package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gyro {
    public static BHI260IMU imu;
    private static double resetAngle;
    private static double lastAngle;



    public static void init(HardwareMap hardwareMap) {/*
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters();
        parameters.angleUnit = BHI260IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BHI260IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BHI260IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);
        */
    }

    public static void resetGyro(){
        //resetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public static double getAngle() {
//        telemetry.addData("angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngle );
        //return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - resetAngle;\
        return 0;
    }



    public static double getDeltaAngle(){
        return getAngle() - lastAngle;
    }

    public static void setLastAngle(double angle){
        lastAngle = angle;
    }


}