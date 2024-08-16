package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuManager {
    private IMU imu;

    private double lastAngle = 0; //safety in case the imu fails

    private boolean imuInitError = false;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public void initImu(){
        try {
            // Initializing imu to avoid errors
            imu = hardwareMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();
            imuInitError = false;
        } catch (Exception e) {
            imuInitError = true;
            telemetry.addData("imu Init error", true);
        }
    }

    public void initImu(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initImu();
    }
    public double getYawRadians(){
        if (imuInitError) {
            initImu();
        }

        try {
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            lastAngle = angle;
            return lastAngle;
        } catch (Exception e){
            telemetry.addData("imu error", true);
            return 0;
        }

    }
}
