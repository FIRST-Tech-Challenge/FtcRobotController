package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="TankAutoOP", group="Linear OpMode")
@Disabled
public class TankDriveAutonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDC = null;
    private DcMotor rightDC = null;
    private Servo servo1 = null;
    private BNO055IMU imu = null;
    private static final double TICKS_PER_REVOLUTION = 288;
    private static final double WHEEL_DIAMETER_INCHES = 90/25.4; //for the fat REV wheels
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION*GEAR_RATIO)/(WHEEL_DIAMETER_INCHES*Math.PI);

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit          = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit          = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftDC = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDC = hardwareMap.get(DcMotor.class, "rightMotor");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        waitForStart();

        encoderDrive(1, 5, 5, 2500);
        encoderDrive(0.5, -5, 5, 1000);
        servoMove(0.8, 5000, servo1);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(double speed, double left, double right, long pause_time) {
        if (opModeIsActive()) {
            int leftTarget = (int)(leftDC.getCurrentPosition()+left*TICKS_PER_INCH);
            int rightTarget = (int)(leftDC.getCurrentPosition()+right*TICKS_PER_INCH);

            leftDC.setTargetPosition(leftTarget);
            rightDC.setTargetPosition(rightTarget);

            leftDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDC.setPower(Math.abs(speed));
            rightDC.setPower(Math.abs(speed));

            leftDC.setPower(0);
            rightDC.setPower(0);

            leftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(pause_time);
        }
    }

    public void servoMove(double position, long pause_time, Servo servo) {
        servo.setPosition(position);
        sleep(pause_time);
    }
}