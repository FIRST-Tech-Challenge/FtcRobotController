/*
    TODO: Add movement methods
    TODO: Add camera
    TODO: Add sensors (gyro, distance)
 */

package org.firstinspires.ftc.teamcode;

/*
    Imports
 */

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class RobotClass {

    //initializing variables
    private LinearOpMode myOpMode = null;

    //defining motor variables
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public BNO055IMU imu;
    public Orientation angles;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public RobotClass (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init (HardwareMap hardwareMap) {
        //initializing motors
        initMotors(hardwareMap);

        //initializing IMU
        try {
            initGyro(hardwareMap);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //initializing camera
        initCamera(hardwareMap);
    }

    //initalizing motors
    private void initMotors (HardwareMap hardwareMap) {
        //assigning motor variables to configuration name
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //setting direction of motors
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //setting zero power behavior to brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //initializing IMU
    private void initGyro (HardwareMap hardwareMap) throws InterruptedException{
        //TODO: Test different IMU configs

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //hardware mapping and initializing imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){
            sleep(10);
        }
        myOpMode.telemetry.addData("Status", imu.isGyroCalibrated());
        myOpMode.telemetry.update();
    }

    //initializing camera
    private void initCamera (HardwareMap hardwareMap) {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    //resetting encoders
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //stops all motors
    public void stopMotors() {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setting power to 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveWithoutEncoders (double powerLeft, double powerRight, int timeInMs) throws InterruptedException {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //setting power of motors
        frontLeft.setPower(powerLeft);
        frontRight.setPower(powerRight);
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
        //waiting while running motors
        sleep(timeInMs);
        //stops motors
        stopMotors();
    }

    //Moving using encoders
    public void moveWithEncoders (double power, double cm) throws InterruptedException {
        //setting number of ticks per 10 cm to get number of ticks per cm
        int ticksPer10cm = 64;
        int ticksPerCm = ticksPer10cm / 10;
        int target = (int) Math.round(cm * ticksPerCm);

        //resetting
        resetEncoders();

        //setting target position of motors
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        //waiting while the motors are busy
        while (frontLeft.isBusy()){
            sleep(50);
        }

        //stopping all motors
        stopMotors();
    }

    //turning with gyro code
    public void gyroTurning (double targetAngleDegrees) throws InterruptedException {
        boolean run = true;
        while (run) {
            angles = imu.getAngularOrientation();
            //using gyro
            if (angles.firstAngle >= targetAngleDegrees - 0.5 && angles.firstAngle <= targetAngleDegrees + 0.5) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500);
                if (angles.firstAngle >= targetAngleDegrees - 0.5 && angles.firstAngle <= targetAngleDegrees + 0.5) {
                    run = false;
                    return;
                }
            } else if (angles.firstAngle >= targetAngleDegrees) {
                if (angles.firstAngle <= targetAngleDegrees + 10) {
                    frontLeft.setPower(0.15);
                    frontRight.setPower(-0.15);
                    backLeft.setPower(0.15);
                    backRight.setPower(-0.15);
                } else {
                    frontLeft.setPower(0.25);
                    frontRight.setPower(-0.25);
                    backLeft.setPower(0.25);
                    backRight.setPower(-0.25);
                }
            } else if (angles.firstAngle <= targetAngleDegrees) {
                if (angles.firstAngle >= targetAngleDegrees - 10) {
                    frontLeft.setPower(-0.15);
                    frontRight.setPower(0.15);
                    backLeft.setPower(-0.15);
                    backRight.setPower(0.15);

                } else {
                    frontLeft.setPower(-0.25);
                    frontRight.setPower(0.25);
                    backLeft.setPower(-0.25);
                    backRight.setPower(0.25);
                }
            }
            stopMotors();
        }
    }

    //strafing class with power and direction as parameters
    public void strafing (String direction, double power, int timeInMs) throws InterruptedException {
        if (direction == "left") {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        } else if (direction == "right"){
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        } else {
            myOpMode.telemetry.addData("Error", "Invalid direction");
            myOpMode.telemetry.update();
        }
        sleep(timeInMs);
        stopMotors();
    }

    //Using AprilTag to find the team prop
    public int findTeamProp (int tagID) {
        //initiallzes current detection variable
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        //defines bounds of ROIs
        int [] leftROI = new int[2];
        int [] rightROI = new int[2];
        int [] centerROI = new int[2];

        //TODO: Define ROIs
        leftROI[0] = 0;
        leftROI[1] = 0;

        rightROI[0] = 0;
        rightROI[1] = 0;

        centerROI[0] = 0;
        centerROI[1] = 0;

        //goes through a for loop of all detections and will search for the id 502.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                if((detection.ftcPose.x > leftROI[0]) ||(detection.ftcPose.x < leftROI[1])){
                    return 1;
                } else if ((detection.ftcPose.x > rightROI[0]) ||(detection.ftcPose.x < rightROI[1])){
                    return 3;
                } else if ((detection.ftcPose.x > centerROI[0]) ||(detection.ftcPose.x < centerROI[1])){
                    return 2;
                } else {
                    return 4;
                }
            } else {
                return 4;
            }
        }
        return 0;
    }
}
