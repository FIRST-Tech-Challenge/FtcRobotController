package org.firstinspires.ftc.teamcode.TeleOps.Memdev.Zayne;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ZayneTest {

    private DcMotor[] motoro;//Why the wierd names?
    private IMU iymu;
    private double currentAngle = 0;

    private double angle = 0;//This is never used, why is it here?
    private int zz=0;
    private int oo=1;
    private int tt=2;
    private int t=3;

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot (
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
    );

    public ZayneTest(DcMotor[] mot, IMU imu){
        motoro = mot;
        iymu = imu;
        for (DcMotor m : motoro) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //iymu.initialize(parameters);
    }

    public void movo(double x, double y, double rx){

        currentAngle = iymu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Mecanum math
        double rotX = x * Math.cos(currentAngle) - y * Math.sin(currentAngle);
        double rotY = x * Math.sin(currentAngle) + y * Math.cos(currentAngle);

        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flpwr = (rotY + rotX + rx) / d;
        double blpwr = (rotY - rotX + rx) / d;
        double frpwr = (rotY - rotX - rx) / d;
        double brpwr = (rotY + rotX - rx) / d;

        motoro[zz].setPower(frpwr); // 2
        motoro[oo].setPower(flpwr); // 0
        motoro[tt].setPower(brpwr); // 3
        motoro[t].setPower(blpwr); // 1


    }


}
