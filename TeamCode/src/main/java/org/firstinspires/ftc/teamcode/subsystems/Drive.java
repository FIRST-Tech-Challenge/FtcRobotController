package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drive extends SubsystemBase {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private Telemetry t;
//    private BNO055IMU imu;


    public Drive(HardwareMap hardwareMap, Telemetry t){
        this.t = t;


//        leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
//        leftRear = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
//        rightRear = hardwareMap.get(DcMotorEx.class, "BackRightMotor");
//        rightFront = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");

        leftFrontMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);

    }

    public void arcadeDrive(double forward, double turn, double strafe, boolean curve) {
        if (curve) {
            forward = inputCurve(forward);
            turn = inputCurve(turn);
            strafe = inputCurve(strafe);
        }

        //Calculate speed for each motor
        double frontLeft = forward + turn + strafe;
        double frontRight = forward - turn - strafe;
        double backLeft = forward + turn - strafe;
        double backRight = forward - turn + strafe;

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set motor
        leftFrontMotor.setPower(frontLeft);
        leftBackMotor.setPower(backLeft);
        rightFrontMotor.setPower(frontRight);
        rightBackMotor.setPower(backRight);
    }

    public void stop() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double inputCurve(double input) {
        if (0 <= input) {
            // input is positive
            return input * input;
        } else {
            // input is negative
            return -(input * input);
        }

    }
    public void runToPosition(double distanceInches, double power){
        int distanceCounts = getCountFromInches(distanceInches);
        leftFrontMotor.setTargetPosition(distanceCounts);
        leftBackMotor.setTargetPosition(distanceCounts);
        rightFrontMotor.setTargetPosition(distanceCounts);
        rightBackMotor.setTargetPosition(distanceCounts);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set motor
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }
    public void runToPositionStrafe(int distanceCounts, double power){
        leftFrontMotor.setTargetPosition(distanceCounts);
        leftBackMotor.setTargetPosition(-distanceCounts);
        rightFrontMotor.setTargetPosition(-distanceCounts);
        rightBackMotor.setTargetPosition(distanceCounts);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set motor
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }
    public double getLeftFrontMotorInches(){
        return getInchesFromCount(leftFrontMotor.getCurrentPosition());
    }
    public double getInchesFromCount(double count){
        return count/44.31372549;
    }
    public int getCountFromInches (double inches){
        return (int) (inches*44.31372549);
    }
    public int[] getAllEncoderCounts() {
        int[] counts = new int[4];

        counts[0] = leftFrontMotor.getCurrentPosition();
        counts[1] = leftBackMotor.getCurrentPosition();
        counts[2] = rightFrontMotor.getCurrentPosition();
        counts[3] = rightBackMotor.getCurrentPosition();

        return counts;
    }
    public double getHeading (){
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return 0;
    }
}