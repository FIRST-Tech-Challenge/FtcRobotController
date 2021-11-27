package org.firstinspires.ftc.masters;//package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


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

    public DcMotor carousel;

    LinearOpMode opmode;
    HardwareMap hardwareMap;
    String color;


    OpenCvWebcam webcam;
    EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline pipeline;

    public RobotClass(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode, String color) {
        this.hardwareMap= hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft" );
        frontRight = hardwareMap.get(DcMotor.class, "frontRight" );
        backLeft = hardwareMap.get(DcMotor.class, "backLeft" );
        backRight = hardwareMap.get(DcMotor.class, "backRight" );
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"colorSensorRight");
        colorSensorMiddle = hardwareMap.get(RevColorSensorV3.class,"colorSensorMiddle");
        carousel = hardwareMap.get(DcMotor.class, "carousel");

        this.opmode= opmode;

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       // parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm=null;//= new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

    public void parkBlue () {
        wayneStrafeBlue(0.2);
        if (colorSensorRight.blue()>120) {
            strafeRight(0.2, 0.35);
        }
        if (colorSensorLeft.blue()>120) {
            strafeLeft(0.2,0.35);
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
            telemetry.addData("Current Front Left Motor Position: ", frontLeft.getCurrentPosition());
            telemetry.addData("Current Front Right Motor Position: ", frontRight.getCurrentPosition());
            telemetry.addData("Current Back Left Motor Position: ", backLeft.getCurrentPosition());
            telemetry.addData("Current Back Right Motor Position: ", backRight.getCurrentPosition());
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

    public void mecanumWitchcraftBackLeft (double speed, double rotations) {
        int frontLeftCurrent = frontLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionFrontLeft = frontLeftCurrent - rotations*ticks;
        double toPositionBackRight = backRightCurrent - rotations*ticks;

        frontLeft.setTargetPosition((int)toPositionFrontLeft);
        backRight.setTargetPosition((int)toPositionBackRight);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(-speed);
        backRight.setPower(-speed);

        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && backRight.isBusy() )) {

            // Display it for the driver.
            motorTelemetry();
        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public double getIntegratedHeading() {
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
        motorSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentHeading = getIntegratedHeading();

        while (currentHeading > targetHeading + tolerance || currentHeading < targetHeading - tolerance) {
            currentHeading = getIntegratedHeading();
            telemetry.addData("Turning to: ", 90);
            telemetry.addData("Current Heading: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("Integrated Heading: ", getIntegratedHeading());
            if (currentHeading > targetHeading) {
                if (abs(currentHeading - targetHeading) > (tolerance+1)*2) {
                    setSpeedForTurnLeft(speed);
                } else if (abs(currentHeading - targetHeading) <= tolerance*2) {
                    setSpeedForTurnLeft(speed/2);
                }
            } else if (currentHeading < targetHeading) {
                if (abs(currentHeading - targetHeading) > tolerance*2) {
                    setSpeedForTurnRight(speed);
                } else if (abs(currentHeading - targetHeading) <= (tolerance+1)*2) {
                    setSpeedForTurnRight(speed/2);
                }
            }
        }
        stopMotors();
    }

    public void turnToHeadingSloppy (double speed, double targetHeading, double slowAt) { // -175 175
        motorSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (targetHeading > 0) {
            setSpeedForTurnLeft(speed);
            while (getAngleFromGyro() < targetHeading) {
                telemetry.addData("Current Angle: ",getAngleFromGyro());
                telemetry.addData("Target Heading: ",targetHeading);
                telemetry.update();
                if (getAngleFromGyro() > targetHeading-slowAt) {
                    setSpeedForTurnLeft(.1);
                }
            }
        } else if (targetHeading < 0) {
            setSpeedForTurnRight(speed);
            while (getAngleFromGyro() > targetHeading) {
                telemetry.addData("Current Angle: ",getAngleFromGyro());
                telemetry.addData("Target Heading: ",targetHeading);
                telemetry.update();
                if (getAngleFromGyro() < targetHeading+slowAt) {
                    setSpeedForTurnRight(.1);
                }
            }
        }
        stopMotors();
    }
/*
if 360-abs(currentHeading)-abs(targetHeading) > 180

 */

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
    public void openCVInnitShenanigans() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline(telemetry);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        // webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Can't open camera");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public enum RingPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        public SkystoneDeterminationPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(45, 230);
        static final Point REGION2_TOP_LEFT_ANCHOR_POINT = new Point(285, 250);
        static final Point REGION3_TOP_LEFT_ANCHOR_POINT = new Point(603, 280);


        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 42;

        final int FREIGHT_PRESENT_THRESHOLD = 110;

        Point region1_pointA = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x,
                REGION1_TOP_LEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOP_LEFT_ANCHOR_POINT.x,
                REGION2_TOP_LEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region3_pointA = new Point(
                REGION3_TOP_LEFT_ANCHOR_POINT.x,
                REGION3_TOP_LEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_A;
        Mat region2_A;
        Mat region3_A;
        Mat LAB = new Mat();

        Mat A = new Mat();
        int avg1 = 0;
        int avg2 = 0;
        int avg3 = 0;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition position = EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts the A channel to the 'A' variable*/

        void inputToLAB_A(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToLAB_A(firstFrame);

            region1_A = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_A = A.submat(new Rect(region2_pointA, region2_pointB));
            region3_A = A.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToLAB_A(input);
            region1_A = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_A = A.submat(new Rect(region2_pointA, region2_pointB));
            region3_A = A.submat(new Rect(region3_pointA, region3_pointB));

            avg1 = (int) Core.mean(region1_A).val[0];
            avg2 = (int) Core.mean(region2_A).val[0];
            avg3 = (int) Core.mean(region3_A).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            if (avg1 < FREIGHT_PRESENT_THRESHOLD) {
                position = EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.LEFT;
            } else if (avg2 < FREIGHT_PRESENT_THRESHOLD) {
                position = EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.MIDDLE;
            } else {
                position = EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.RIGHT;
            }
            telemetry.addData("Analysis", avg1);
            telemetry.addData("Analysis2", avg2);
            telemetry.addData("Analysis3", avg3);
            telemetry.addData("Position", position);
            telemetry.update();


            return input;
        }

        public int getAnalysis() {
            return avg1;
        }

        public int getAnalysis2() {
            return avg2;
        }

        public int getAnalysis3() {
            return avg3;
        }


    }
        public EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition analyze() {
        pipeline.getAnalysis();
        return pipeline.position;
    }

}