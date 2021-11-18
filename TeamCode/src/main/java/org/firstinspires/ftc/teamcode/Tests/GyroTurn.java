package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.CompBotV3.CompBotV3.distanceK;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.SimpleBlueVisionYCbCr;
import org.firstinspires.ftc.teamcode.vision.SimpleExtraVisionYCbCr;
import org.firstinspires.ftc.teamcode.vision.SimpleRedVisionYCbCr;
import org.openftc.easyopencv.OpenCvCamera;

public class GyroTurn extends LinearOpMode {
    public DcMotor fl = null, fr = null, bl = null, br = null;
    public RevIMU imu = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // init
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = new RevIMU(hardwareMap,"imu");
        imu.init(parameters);

        // method
        double speed = 0.5, angle = 90;
        long time = 5000;
        gyroTurn(speed,angle,time);
    }

    public void gyroTurn(double speed, double angle, long time){
        double expected = imu.getHeading() + angle, tolerance = 0.01, error, coef = 0.1;
        while(expected > 180){
            expected -= 360;
        }
        while (expected < -180){
            expected += 360;
        }
        ElapsedTime timer = new ElapsedTime();
        do{
            error = -(imu.getHeading() - expected);
            fl.setPower(coef*error*speed);
            bl.setPower(coef*error*speed);
            fr.setPower(-coef*error*speed);
            br.setPower(-coef*error*speed);
        }while(Math.abs(error) > tolerance && timer.milliseconds() < time);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    public void encoder(double forward, double strafe, long time, double forwardSpeed, double strafeSpeed){
        fl.setTargetPosition((int) (fl.getCurrentPosition() + distanceK * (forward + strafe)));
        br.setTargetPosition((int) (br.getCurrentPosition() + distanceK * (forward + strafe)));
        bl.setTargetPosition((int) (fl.getCurrentPosition() + distanceK * (forward-strafe)));
        fr.setTargetPosition((int) (fl.getCurrentPosition() + distanceK * (forward-strafe)));

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime();
        while ((fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) && timer.milliseconds() < time){
            fl.setPower(forwardSpeed + strafeSpeed);
            fr.setPower(forwardSpeed + strafeSpeed);
            bl.setPower(forwardSpeed - strafeSpeed);
            br.setPower(forwardSpeed - strafeSpeed);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
}
