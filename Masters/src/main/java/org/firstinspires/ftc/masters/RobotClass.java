package org.firstinspires.ftc.masters;//package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Date;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


public class RobotClass {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    private double ticks = 537;//537
    private double ticksTheSequel = 2786;
    BNO055IMU imu;

    public Telemetry telemetry;
    RevColorSensorV3 colorSensorLeft;
    RevColorSensorV3 colorSensorRight;
    RevColorSensorV3 colorSensorMiddle;

    LinearOpMode opmode;
    HardwareMap hardwareMap;
    String color;

    public RobotClass(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode, String color) {
        this.hardwareMap= hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft" );
        frontRight = hardwareMap.get(DcMotor.class, "frontRight" );
        backLeft = hardwareMap.get(DcMotor.class, "backLeft" );
        backRight = hardwareMap.get(DcMotor.class, "backRight" );
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"colorSensorRight");
        colorSensorMiddle = hardwareMap.get(RevColorSensorV3.class,"colorSensorMiddle");

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;
        this.opmode = opmode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       // parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm=null;//= new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu1");
        imu.initialize(parameters);

    }

    public RobotClass(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode) {
        this.hardwareMap= hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft" );
        frontRight = hardwareMap.get(DcMotor.class, "frontRight" );
        backLeft = hardwareMap.get(DcMotor.class, "backLeft" );
        backRight = hardwareMap.get(DcMotor.class, "backRight" );

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"colorSensorRight");
        colorSensorMiddle = hardwareMap.get(RevColorSensorV3.class,"colorSensorMiddle");

        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm=null;//= new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu1");
        imu.initialize(parameters);
        this.opmode = opmode;


    }

    public void testGyro(){
        while(opmode.opModeIsActive()){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            telemetry.addData("gravity",imu.getGravity().toString());
            telemetry.addData("1",angles.firstAngle);
            telemetry.addData("2", angles.secondAngle);
            telemetry.addData("3", angles.thirdAngle);
            telemetry.update();
        }
    }

    public double getAngleFromGyro() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    public void forward (double speed, double rotations){
        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        telemetry.addData("Target Front Left Motor Position", leftCurrent);
        telemetry.addData("Target Front Right Motor Position", rightCurrent);
        telemetry.addData("Target Back Left Motor Position", backLeftCurrent);
        telemetry.addData("Target Back Right Motor Position", backRightCurrent);
        telemetry.update();
//        try {
//
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

        double toPositionLeft = leftCurrent + rotations*ticks;
        double toPositionRight = rightCurrent + rotations*ticks;
        double toPositionbackLeft = backLeftCurrent + rotations*ticks;
        double toPositionbackRight = backRightCurrent + rotations*ticks;

        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
        telemetry.addData("Target Front Right Motor Position", toPositionRight);
        telemetry.addData("Target Back Left Motor Position", toPositionbackLeft);
        telemetry.addData("Target Front Left Motor Position", toPositionbackLeft);
        telemetry.update();

        frontLeft.setTargetPosition((int)toPositionLeft);
        frontRight.setTargetPosition((int)toPositionRight);
        backLeft.setTargetPosition((int)toPositionbackLeft);
        backRight.setTargetPosition((int)toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(speed));
        frontRight.setPower(abs(speed));
        backLeft.setPower(abs(speed));
        backRight.setPower(abs(speed));

        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Display it for the driver.
            motorTelemetry();
        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void backwards (double speed, double rotations) {
        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent - rotations * ticks;
        double toPositionRight = rightCurrent - rotations * ticks;
        double toPositionbackLeft = backLeftCurrent - rotations * ticks;
        double toPositionbackRight = backRightCurrent - rotations * ticks;

        frontLeft.setTargetPosition((int) toPositionLeft);
        frontRight.setTargetPosition((int) toPositionRight);
        backLeft.setTargetPosition((int) toPositionbackLeft);
        backRight.setTargetPosition((int) toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

//        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
//        telemetry.addData("Target Front Right Motor Position", toPositionRight);
//        telemetry.addData("Target Back Left Motor Position", toPositionBackLeft);
//        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
//        telemetry.update();
    }

    public void wayneStrafeBlue (double speed) {

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        while (colorSensorRight.blue()< 120 && colorSensorLeft.blue()<120) {
            telemetry.addData("Right blue ", colorSensorRight.blue());
            telemetry.addData("Left blue ", colorSensorLeft.blue());
            telemetry.update();
        }

        stopMotors();
    }

    public void wayneStrafeRed (double speed) {

        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        while (colorSensorRight.red()< 120 && colorSensorLeft.red()<120) {
            telemetry.addData("Right red ", colorSensorRight.red());
            telemetry.addData("Left red ", colorSensorLeft.red());
            telemetry.update();
        }

        stopMotors();
    }

    public void parkRed () {

        wayneStrafeRed(0.2);
        if (colorSensorLeft.red()>120) {
            strafeLeft(0.2, 0.35);
        }
        if (colorSensorRight.red()>120) {
            strafeRight(0.2,0.35);
        }
        forward(0.2,-1);

    }

        public void setSpeedForTurnRight (double speed) {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);
        }

        public void setSpeedForTurnLeft (double speed) {
            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
        }

        public void stopMotors () {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        protected void motorTelemetry(){
            telemetry.addData("Target Front Left Motor Position", frontLeft.getCurrentPosition());
            telemetry.addData("Target Front Right Motor Position", frontRight.getCurrentPosition());
            telemetry.addData("Target Back Left Motor Position", backLeft.getCurrentPosition());
            telemetry.addData("Target Back Right Motor Position", backRight.getCurrentPosition());
            telemetry.update();
        }


    public void pivotRightSloppy (double speed, double angle) {
        setSpeedForTurnRight(speed);

        double targetAngle = getAngleFromGyro() - angle;

        while (getAngleFromGyro() > targetAngle && opmode.opModeIsActive()) {
            telemetry.addData("Gyro Angle: ", getAngleFromGyro());
            telemetry.update();
        }

        stopMotors();

        telemetry.addData("Gyro Angle", getAngleFromGyro());
        telemetry.update();
    }

    public void pivotLeftSloppy (double speed, double angle) {
        setSpeedForTurnLeft(speed);

        double targetAngle = getAngleFromGyro() + angle;

        while (getAngleFromGyro() < targetAngle && opmode.opModeIsActive()) {
            telemetry.addData("Gyro Angle: ", getAngleFromGyro());
            telemetry.update();
        }

        stopMotors();

        telemetry.addData("Gyro Angle", getAngleFromGyro());
        telemetry.update();
    }

    public void pivotRight (double speed, double angle) {
        double targetAngle = getAngleFromGyro() - angle;
        pivotRightSloppy(speed, angle);

        telemetry.addData("Middle Gyro Angle: ", getAngleFromGyro());
        telemetry.update();

        speed= speed*0.5;
        if (getAngleFromGyro()<targetAngle-0.5) {
            setSpeedForTurnLeft(speed);
            while (getAngleFromGyro() < targetAngle && opmode.opModeIsActive()) {
                telemetry.addData("Gyro Angle: ", getAngleFromGyro());
                telemetry.update();
            }
        }

        stopMotors();
        telemetry.addData("Completed Gyro Angle: ", getAngleFromGyro());
        telemetry.update();
    }
    public void pivotLeft (double speed, double angle) {
        double targetAngle = getAngleFromGyro() + angle;
        pivotLeftSloppy(speed, angle);

        telemetry.addData("Middle Gyro Angle: ", getAngleFromGyro());
        telemetry.update();

        speed= speed*0.5;
        setSpeedForTurnRight(speed);
        if (getAngleFromGyro()>targetAngle+0.5) {
            while (getAngleFromGyro() > targetAngle + 0.5 && opmode.opModeIsActive()) {
                telemetry.addData("Gyro Angle: ", getAngleFromGyro());
                telemetry.update();
            }
        }

        stopMotors();
        telemetry.addData("Completed Gyro Angle: ", getAngleFromGyro());
        telemetry.update();
    }

    public void mecanumWitchcraft (double degree, double time) {
        double x = java.lang.Math.cos(degree);
        double y = java.lang.Math.sin(degree);

        frontLeft.setPower(y + x);
        backLeft.setPower(y - x);
        frontRight.setPower(y - x);
        backRight.setPower(y + x);


        stopMotors();

    }
    public void strafeLeft (double speed, double rotations) {

        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent - rotations*ticks;
        double toPositionRight = rightCurrent + rotations*ticks;
        double toPositionbackLeft = backLeftCurrent + rotations*ticks;
        double toPositionbackRight = backRightCurrent - rotations*ticks;

        frontLeft.setTargetPosition((int)toPositionLeft);
        frontRight.setTargetPosition((int)toPositionRight);
        backLeft.setTargetPosition((int)toPositionbackLeft);
        backRight.setTargetPosition((int)toPositionbackRight);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(-speed));
        frontRight.setPower(abs(speed));
        backLeft.setPower(abs(speed));
        backRight.setPower(abs(-speed));
        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Display it for the driver.
            motorTelemetry();
        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void strafeRight (double speed, double rotations) {

        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent + rotations*ticks;
        double toPositionRight = rightCurrent - rotations*ticks;
        double toPositionbackLeft = backLeftCurrent - rotations*ticks;
        double toPositionbackRight = backRightCurrent + rotations*ticks;

        frontLeft.setTargetPosition((int)toPositionLeft);
        frontRight.setTargetPosition((int)toPositionRight);
        backLeft.setTargetPosition((int)toPositionbackLeft);
        backRight.setTargetPosition((int)toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(speed));
        frontRight.setPower(abs(-speed));
        backLeft.setPower(abs(-speed));
        backRight.setPower(abs(speed));
        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Display it for the driver.
            motorTelemetry();
        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double previousHeading = 0; //Outside of method
    private double integratedHeading = 0;

    /**
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     * @return The integrated heading on the interval (-inf, inf).
     */
    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }


    public void turnToHeading (double speed, double targetHeading, int tolerance) {
        double currentHeading = getIntegratedHeading();

        while (currentHeading > targetHeading + tolerance && currentHeading < targetHeading - tolerance) {
            currentHeading = getIntegratedHeading();
            if (currentHeading > targetHeading) {
                if (abs(currentHeading - targetHeading) > (tolerance+1)*2) {
                    setSpeedForTurnLeft(speed);
                } else if (abs(currentHeading - targetHeading) <= tolerance*2) {
                    setSpeedForTurnLeft(speed/2);
                }
            } else if (currentHeading < targetHeading) {
                if (abs(currentHeading - targetHeading) > tolerance*2) {
                    setSpeedForTurnLeft(speed);
                } else if (abs(currentHeading - targetHeading) <= (tolerance+1)*2) {
                    setSpeedForTurnLeft(speed/2);
                }
            }
        }
        stopMotors();
    }



    protected void motorSetMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }
//
    public void pause(int millis){
        long startTime = new Date().getTime();
        long time = 0;

        while (time<millis && opmode.opModeIsActive()) {
            time = new Date().getTime() - startTime;
        }
    }


    public void pauseButInSecondsForThePlebeians(double seconds) {
        long startTime = new Date().getTime();
        long time = 0;

        while (time<seconds*1000 && opmode.opModeIsActive()) {
            time = new Date().getTime() - startTime;
        }
    }
    }