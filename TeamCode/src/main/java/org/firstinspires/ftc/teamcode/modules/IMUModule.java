package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

public class IMUModule {
    private final OpModeUtilities opModeUtilities;
    IMU imu;

    public IMUModule(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        this.imu = opModeUtilities.getHardwareMap().get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();
    }

    public IMU getIMU() {
        return imu;
    }
}
