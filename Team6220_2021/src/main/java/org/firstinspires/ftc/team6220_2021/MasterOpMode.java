package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2021.ResourceClasses.DriverInput;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class MasterOpMode extends LinearOpMode {
    // Motors
    public static DcMotor motorFrontLeft;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackLeft;
    public static DcMotor motorBackRight;

    // Other Devices
    // servos go here

    // Create Drivers
    public DriverInput driver1;
    public DriverInput driver2;

    // IMUs
    public BNO055IMU imu;

    // Webcam
    OpenCvCamera webcam;

    // Initializes the motors, servos, IMUs, and drivers
    public void Initialize() {
        // Drive train motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        motorBackRight = hardwareMap.dcMotor.get("motorBR");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Retrieve and initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    // Pauses for time milliseconds
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}