package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    public static BHI260IMU imu;
    private static double resetAngle;
    private static double lastAngle;



    public static void init(HardwareMap hardwareMap) {
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        /*
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                new Orientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        AngleUnit.DEGREES,
                        90,
                        0,
                        90,
                        0  // acquisitionTime, not used
                )
        ));
         */
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);

    }

    public static void resetGyro(){
        resetAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public static double getAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle - resetAngle;

    }



    public static double getDeltaAngle(){
        return getAngle() - lastAngle;
    }

    public static void setLastAngle(double angle){
        lastAngle = angle;
    }


}