package org.firstinspires.ftc.blackswan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

import android.graphics.Color;

import java.util.Date;

public class Robot {
    Telemetry telemetry;
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotor linearSlide;
    Servo clawservo;
    RevColorSensorV3 colorSensorL, colorSensorR;
    BNO055IMU imu;
    LinearOpMode opMode;

    final int TICKS_PER_ROTATION = 537;

    public void stopMotors() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "backRight");

        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensorL = hardwareMap.get(RevColorSensorV3.class, "colorSensorL");
        colorSensorR = hardwareMap.get(RevColorSensorV3.class, "colorSensorR");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;//= new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public void forward(double distance, double speed) {

        double distanceInTicks = distance * TICKS_PER_ROTATION;

        int frontLeftPosition = motorFrontLeft.getCurrentPosition();

        int frontLeftTarget = frontLeftPosition + (int) distanceInTicks;

        int frontRightPosition = motorFrontRight.getCurrentPosition();

        int backLeftPosition = motorBackLeft.getCurrentPosition();

        int backLeftTarget = backLeftPosition + (int) distanceInTicks;

        int backRightPosition = motorBackRight.getCurrentPosition();

        int backRightTarget = backRightPosition + (int) distanceInTicks;

        int frontRightTarget = frontRightPosition + (int) distanceInTicks;

        motorFrontLeft.setTargetPosition((int) frontLeftTarget);
        motorFrontRight.setTargetPosition((int) frontRightTarget);
        motorBackLeft.setTargetPosition((int) backLeftTarget);
        motorBackRight.setTargetPosition((int) backRightTarget);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(abs(speed));
        motorFrontRight.setPower(abs(speed));
        motorBackLeft.setPower(abs(speed));
        motorBackRight.setPower(abs(speed));

        while (this.opMode.opModeIsActive() &&
                (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy())) {

            // Display it for the driver.

        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void back(double distance, double speed) {

        forward(-distance, speed);
    }

    public void left(double distance, double speed) {

        double distanceInTicks = distance * TICKS_PER_ROTATION;

        int frontLeftPosition = motorFrontLeft.getCurrentPosition();

        int frontLeftTarget = frontLeftPosition - (int) distanceInTicks; //

        int frontRightPosition = motorFrontRight.getCurrentPosition();

        int backLeftPosition = motorBackLeft.getCurrentPosition();

        int backLeftTarget = backLeftPosition + (int) distanceInTicks;

        int backRightPosition = motorBackRight.getCurrentPosition();

        int backRightTarget = backRightPosition - (int) distanceInTicks; //

        int frontRightTarget = frontRightPosition + (int) distanceInTicks;


        motorFrontLeft.setTargetPosition((int) frontLeftTarget);
        motorFrontRight.setTargetPosition((int) frontRightTarget);
        motorBackLeft.setTargetPosition((int) backLeftTarget);
        motorBackRight.setTargetPosition((int) backRightTarget);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(abs(speed));
        motorFrontRight.setPower(abs(speed));
        motorBackLeft.setPower(abs(speed) * -1);
        motorBackRight.setPower(abs(speed) * -1);

        while (this.opMode.opModeIsActive() &&
                (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy())) {
            telemetry.addData("frontLeftPosition", frontLeftPosition);
            telemetry.addData("frontLeftTarget", frontLeftTarget);
            telemetry.addData("frontRightPosition", frontRightPosition);
            telemetry.addData("frontRightTarget", frontRightTarget);
            telemetry.addData("backLeftPosition", backLeftPosition);
            telemetry.addData("backLeftTarget", backLeftTarget);
            telemetry.addData("backRightPosition", backRightPosition);
            telemetry.addData("backRightTarget", backRightTarget);
            telemetry.update();
            // Display it for the driver.

        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public double gyroAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    public void right(double distance, double speed) {
        left(-distance, speed);
    }

    public void turnRight(double angle, double speed) {

        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(-speed);

        double targetAngle = gyroAngle() + angle;

        while (targetAngle > Math.abs(gyroAngle()) && opMode.opModeIsActive()) {
            telemetry.addData("Current Gyro Angle", gyroAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnLeft(double angle, double speed) {

        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(speed);

        double targetAngle = gyroAngle() + angle;

        while (targetAngle > Math.abs(gyroAngle()) && opMode.opModeIsActive()) {
            telemetry.addData("Current Gyro Angle", gyroAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }
        stopMotors();
    }

    protected void motorSetMode(DcMotor.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorBackLeft.setMode(runMode);
        motorBackRight.setMode(runMode);
    }

    public void pause(int millis) {
        long startTime = new Date().getTime();
        long time = 0;

        while (time < millis && opMode.opModeIsActive()) {
            time = new Date().getTime() - startTime;
        }
    }
}


// HAPPY NEW YEAR
//  *    **  *
//*     *
// SPARKLES
//YIPPEE