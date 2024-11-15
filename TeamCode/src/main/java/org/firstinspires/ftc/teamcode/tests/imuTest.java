package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class imuTest extends OpMode {
    IMU imu;
    FtcDashboard dash;
    Telemetry t2;
    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //CHANGE THESE ONCE ORIENTATION IS KNOW
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();

    }

    @Override
    public void loop() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw();
        double p = imu.getRobotYawPitchRollAngles().getPitch();
        double r = imu.getRobotYawPitchRollAngles().getRoll();
        telemetry.addData("yaw", yaw);
        telemetry.addData("p", p);
        telemetry.addData("r", r);
        t2.addData("yaw", yaw);
        t2.addData("p", p);
        t2.addData("r", r);
        t2.update();
        telemetry.update();



    }
}
