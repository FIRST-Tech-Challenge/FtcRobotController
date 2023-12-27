package org.firstinspires.ftc.masters;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import java.util.Date;

public class robot {
    Telemetry telemetry;
    DcMotor leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor;
    DcMotor linearSlide;
    BNO055IMU imu;
    LinearOpMode opMode;

    final int TICKS_PER_ROTATION = 537;

    public void stopMotors() {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    public robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "backLeft");
        leftRearMotor = hardwareMap.get(DcMotor.class, "frontRight");
        rightRearMotor = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public void forward(double distance, double speed) {

        double distanceInTicks = distance * TICKS_PER_ROTATION;

        int frontLeftPosition = leftFrontMotor.getCurrentPosition();

        int frontLeftTarget = frontLeftPosition + (int) distanceInTicks;

        int frontRightPosition = leftRearMotor.getCurrentPosition();

        int backLeftPosition = rightFrontMotor.getCurrentPosition();

        int backLeftTarget = backLeftPosition + (int) distanceInTicks;

        int backRightPosition = rightRearMotor.getCurrentPosition();

        int backRightTarget = backRightPosition + (int) distanceInTicks;

        int frontRightTarget = frontRightPosition + (int) distanceInTicks;

        leftFrontMotor.setTargetPosition((int) frontLeftTarget);
        leftRearMotor.setTargetPosition((int) frontRightTarget);
        rightFrontMotor.setTargetPosition((int) backLeftTarget);
        rightRearMotor.setTargetPosition((int) backRightTarget);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(abs(speed));
        leftRearMotor.setPower(abs(speed));
        rightFrontMotor.setPower(abs(speed));
        rightRearMotor.setPower(abs(speed));

        while (this.opMode.opModeIsActive() &&
                (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())) {

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

        int frontLeftPosition = leftFrontMotor.getCurrentPosition();

        int frontLeftTarget = frontLeftPosition - (int) distanceInTicks; //

        int frontRightPosition = leftRearMotor.getCurrentPosition();

        int backLeftPosition = rightFrontMotor.getCurrentPosition();

        int backLeftTarget = backLeftPosition + (int) distanceInTicks;

        int backRightPosition = rightRearMotor.getCurrentPosition();

        int backRightTarget = backRightPosition - (int) distanceInTicks; //

        int frontRightTarget = frontRightPosition + (int) distanceInTicks;


        leftFrontMotor.setTargetPosition((int) frontLeftTarget);
        leftRearMotor.setTargetPosition((int) frontRightTarget);
        rightFrontMotor.setTargetPosition((int) backLeftTarget);
        rightRearMotor.setTargetPosition((int) backRightTarget);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(abs(speed));
        leftRearMotor.setPower(abs(speed));
        rightFrontMotor.setPower(abs(speed) * -1);
        rightRearMotor.setPower(abs(speed) * -1);

        while (this.opMode.opModeIsActive() &&
                (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy())) {
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

        leftFrontMotor.setPower(speed);
        leftRearMotor.setPower(-speed);
        rightFrontMotor.setPower(speed);
        rightRearMotor.setPower(-speed);

        double targetAngle = gyroAngle() + angle;

        while (targetAngle > Math.abs(gyroAngle()) && opMode.opModeIsActive()) {
            telemetry.addData("Current Gyro Angle", gyroAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnLeft(double angle, double speed) {

        leftFrontMotor.setPower(-speed);
        leftRearMotor.setPower(speed);
        rightFrontMotor.setPower(-speed);
        rightRearMotor.setPower(speed);

        double targetAngle = gyroAngle() + angle;

        while (targetAngle > Math.abs(gyroAngle()) && opMode.opModeIsActive()) {
            telemetry.addData("Current Gyro Angle", gyroAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }
        stopMotors();
    }

    protected void motorSetMode(DcMotor.RunMode runMode) {
        leftFrontMotor.setMode(runMode);
        leftRearMotor.setMode(runMode);
        rightFrontMotor.setMode(runMode);
        rightRearMotor.setMode(runMode);
    }

    public void pause(int millis) {
        long startTime = new Date().getTime();
        long time = 0;

        while (time < millis && opMode.opModeIsActive()) {
            time = new Date().getTime() - startTime;
        }
    }
}