package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Gyro {
    private static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    private static IMU imu;
    private static float lastAngle = 0;

    public static void init(HardwareMap hardwareMap) {
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)
        );

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    public static void resetGyro() {
        lastAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public static float getAngle(){
        return imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle - lastAngle;
    }

}
