package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBasket2024 extends DriveMethods {

    double startTime = -1;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    @Override
    public void loop() {
        telemetry.addData("code", "running");
        telemetry.addData("time", "%.1f", getRuntime());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());


        if (getRuntime() < 2) {
            omniDrive(0, 1, 0);
        }
        else {
            omniDrive(0, 0, 0);
        }
    }
}
