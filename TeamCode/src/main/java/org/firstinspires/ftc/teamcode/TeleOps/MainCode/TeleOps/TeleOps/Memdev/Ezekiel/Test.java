package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Ezekiel;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Test{
    private DcMotor[] motors;
    private IMU imu;
    private double currentAngle = 0;

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
    );
    public Test(DcMotor[] motors1, IMU imu1) {
        motors = motors1;
        imu = imu1;
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        imu.initialize(parameters);
    }
    public void move(double y, double x, double xr) {

        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotx = x * Math.cos(-currentAngle) - y * Math.sin(-currentAngle);
        double roty = x * Math.sin(-currentAngle) + y * Math.cos(-currentAngle);

        double wheelPower = Math.sqrt((Math.pow(rotx, 2)+Math.pow(roty,2)));
        double wheelAngle = Math.atan2(roty, rotx);
        // pi is in place of encoder stuff I don't know how to do
        double currentWheelAngle = Math.PI;
        double sign = 0;
        double denominator = Math.max(Math.abs(roty) + Math.abs(rotx) + Math.abs(xr), 1);
        double power1 = (wheelPower + xr)/denominator;
        double power2 = (wheelPower - xr)/denominator;
        if (((currentWheelAngle+180)%360) >= wheelAngle) {
            sign = -1;
        } else {
            sign = 1;
        }
        //if (currentWheelAngle != wheelAngle) {
        //    motors[0].setPower(sign);
        //    motors[1].setPower(-sign);
        //    motors[2].setPower(sign);
        //    motors[3].setPower(-sign);
        //} else {
            motors[0].setPower(power1);
            motors[1].setPower(power1);
            motors[2].setPower(power2);
            motors[3].setPower(power2);
        //}
    }
}