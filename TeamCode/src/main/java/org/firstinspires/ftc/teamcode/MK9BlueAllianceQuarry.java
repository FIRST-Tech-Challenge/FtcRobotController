/* ------------------------------
      MK9 Blue Alliance Quarry
---------------------------------*/
package org.firstinspires.ftc.teamcode.Auto;

//---------------------------------------

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import kotlin.Unit;

/* ------------------------------
      Gyro Import Statements
---------------------------------*/
/* ------------------------------
      OpenCV Import Statements
---------------------------------*/
/* ------------------------------
      Road Runner Import Statements
---------------------------------*/


/* ------------------------------
      Start of Code
---------------------------------*/
@Autonomous(name = "Blue Alliance Quarry", group = "Auto GarageBot 15146")
@Disabled
public class MK9BlueAllianceQuarry extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //Wheels
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    //Intake
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    //Left Auto Claw Servos
    private CRServo leftAutoArm = null;
    private Servo leftAutoGripper = null;
    private Servo leftAutoRotate = null;
    //Right Auto Claw Servos
    private CRServo rightAutoArm = null;
    private Servo rightAutoGripper = null;
    private Servo rightAutoRotate = null;
    //Foundation Servos
    private Servo leftFoundation = null;
    private Servo rightFoundation = null;
    //Tape Measure Servo
    private Servo tapeMeasure = null;
    //Encoder Math
    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    /* ------------------------------
          Gyro Sensor Init
    ---------------------------------*/
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    /* ------------------------------
          OpenCV Variables
    ---------------------------------*/
    private static int skystonePos;
    private static int max;
    private static Mat submat1;
    private static Mat submat2;
    private static Mat submat3;
    private static int avg1;
    private static int avg2;
    private static int avg3;
    private static Point pointA = new Point(215, 155);
    private static Point pointB = new Point(225, 145);
    private static Point pointC = new Point(361, 155);
    private static Point pointD = new Point(371, 145);
    private static Point pointE = new Point(510, 155);
    private static Point pointF = new Point(520, 145);
    private final int rows = 640;
    private final int cols = 480;
    OpenCvCamera phoneCam;

    /* ------------------------------
          Start RunOpMode
    ---------------------------------*/
    @Override
    public void runOpMode() throws InterruptedException {
/* ------------------------------
      Init Phone Camera
---------------------------------*/
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
//width, height
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
/* ------------------------------
      Hardware Maps
---------------------------------*/


        //Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //Intake
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        //Set motor directions

        //Wheels MK7
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //Intake
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        //Left Auto Claw
        leftAutoArm = hardwareMap.get(CRServo.class, "leftAutoArm");
        leftAutoGripper = hardwareMap.get(Servo.class, "leftAutoGripper");
        leftAutoRotate = hardwareMap.get(Servo.class, "leftAutoRotate");
        leftAutoArm.setPower(-1);
        leftAutoGripper.setPosition(0);
        leftAutoRotate.setPosition(0.65);

        //Right Auto Claw
        rightAutoArm = hardwareMap.get(CRServo.class, "rightAutoArm");
        rightAutoGripper = hardwareMap.get(Servo.class, "rightAutoGripper");
        rightAutoRotate = hardwareMap.get(Servo.class, "rightAutoRotate");
        rightAutoArm.setPower(1);
        rightAutoGripper.setPosition(1);
        rightAutoRotate.setPosition(0.2);

        //Tape Mesure Shooter'
        tapeMeasure = hardwareMap.get(Servo.class, "tapeMeasure");
        tapeMeasure.setPosition(1);

        //Foundation Servos
        leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
        rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
        leftFoundation.setPosition(0.85);
        rightFoundation.setPosition(0.1);

        //Toggle Break
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                rightBack.getCurrentPosition());

        telemetry.update();
/* ------------------------------
      Gyro Setup
---------------------------------*/
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Set up our telemetry dashboard
        composeTelemetry();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

/* ------------------------------
      Wait For Driver to Start
---------------------------------*/
        //Display SkyStone positon
        if (skystonePos == 1) {
            telemetry.addData("Skystone is in: ", "Middle");
            telemetry.update();
        } else if (skystonePos == 2) {
            telemetry.addData("Skystone is in: ", "Right");
            telemetry.update();
        } else {
            telemetry.addData("Skystone is in: ", "Left");
            telemetry.update();
        }
        telemetry.update();
        waitForStart();
        runtime.reset();

        //HardwareMap for Road Runner
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
/* ------------------------------
    Drive Constants
---------------------------------*/

        DriveConstraints normalConstraint = new DriveConstraints(45, 40, 0, 180, 180, 0);
        DriveConstraints slowConstraint = new DriveConstraints(45, 40, 0, 180, 180, 0);
/* ------------------------------
      Trajectory Builder For Case 0
---------------------------------*/

//        //Straight Tests
//        Trajectory forwardandback = new TrajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), normalConstraint)
//                .lineTo(new Vector2d(-80, 0), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
//                .lineTo(new Vector2d(0, 0), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
//                .lineTo(new Vector2d(-80, 0), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
//                .lineTo(new Vector2d(0, 0), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
//                .build();


        /**
         * LEFT CASE
         */

//        drive.setPoseEstimate(new Pose2d(-33, 63, Math.toRadians(180)));
//        //Move to First Skystone
//        Trajectory diagonalToFirstSkystone_LEFT = new TrajectoryBuilder(new Pose2d(-33, 63, Math.toRadians(180)), normalConstraint)
//                .strafeTo(new Vector2d(-15.5, 27))
//                .strafeTo(new Vector2d(20, 27))
//                .strafeTo(new Vector2d(-15.5, 27))
//                .strafeTo(new Vector2d(-33, 63))
//                .build();

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        //Move to First Skystone
        Trajectory diagonalToFirstSkystone_LEFT = new TrajectoryBuilder(new Pose2d(0, 0, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-120, 0))
                .build();

        //Drive to Foundation first strafe (away from quarry)
        Trajectory foundationFirstTimeFirstStrafe_LEFT = new TrajectoryBuilder(new Pose2d(-15.5, 21, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-15.5, 30))
                .build();

        //Straight-away to Foundation
        Trajectory foundationFirstTimeStraight_LEFT = new TrajectoryBuilder(new Pose2d(-15.5, 30, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(51.5, 30), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(30, 30), () -> {
                    //Bring arm up
                    leftAutoArm.setPower(-0.1);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(45, 30), () -> {
                    //Rotate arm
                    leftAutoRotate.setPosition(0.16);
                    return Unit.INSTANCE;
                })
                .build();

        //Strafe into foundation
        Trajectory foundationFirstTimeSecondStrafe_LEFT = new TrajectoryBuilder(new Pose2d(51.5, 30, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(51.5, 18))
                .build();

        //Drive to Second Skystone (Wall) first strafe
        Trajectory secondSkystoneWallFirstStrafe_LEFT = new TrajectoryBuilder(new Pose2d(51.5, 18, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(51.5, 29))
                .build();

        //Drive to Second Skystone (Corner)
        Trajectory secondSkystoneCorner_LEFT = new TrajectoryBuilder(new Pose2d(51.5, 29, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(-20, 29), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(-10, 29), () -> {
                    //Rotate arm
                    leftFirstStoneArmPrep();
                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(-53, 57))
                .build();

        //Drive to Second Skystone (Block)
        Trajectory secondSkystone_LEFT = new TrajectoryBuilder(new Pose2d(-63, 63, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-41.5, 32))
                .strafeTo(new Vector2d(-41.5, 21))
                .build();

        //Drive to Foundation Long distance
        Trajectory foundationSecondTimeStraight_LEFT = new TrajectoryBuilder(new Pose2d(-41.5, 21, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(-9, 26), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .lineTo(new Vector2d(55, 26), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(6, 26), () -> {
                    //Lower arm slightly
                    leftAutoArm.setPower(0.60);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(31, 26), () -> {
                    //Rotate arm
                    leftAutoRotate.setPosition(0.16);
                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(55, -4))
                .build();

//--------------------
//    Foundation
//--------------------

        //Turn away from foundations (LineTo while Turning)
        Trajectory grabberToFoundation_LEFT = new TrajectoryBuilder(new Pose2d(51, 28, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(45, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                .lineTo(new Vector2d(45, 34), new SplineInterpolator(Math.toRadians(110), Math.toRadians(85)))
                .lineTo(new Vector2d(45, 25), new SplineInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build();

        //Move Foundation
        Trajectory moveFoundation_LEFT = new TrajectoryBuilder(new Pose2d(45, 23, Math.toRadians(90)), normalConstraint)
                .lineTo(new Vector2d(18, 33), new SplineInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build();

        //Push Foundation
        Trajectory pushFoundation_LEFT = new TrajectoryBuilder(new Pose2d(18, 33, Math.toRadians(190)), normalConstraint)
                .lineTo(new Vector2d(52, 67), new SplineInterpolator(Math.toRadians(190), Math.toRadians(180)))
                .build();

        //Park
        Trajectory park_LEFT = new TrajectoryBuilder(new Pose2d(50, 57, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(4, 60), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .build();

        /**
         * CENTER CASE
         */

        //Move to First Skystone
        Trajectory diagonalToFirstSkystone_CENTER = new TrajectoryBuilder(new Pose2d(-33, 63, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-26, 27))
                .strafeTo(new Vector2d(-26, 21))
                .build();

        //Drive to Foundation first strafe (away from quarry)
        Trajectory foundationFirstTimeFirstStrafe_CENTER = new TrajectoryBuilder(new Pose2d(-26, 21, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-26, 30))
                .build();

        //Straight-away to Foundation
        Trajectory foundationFirstTimeStraight_CENTER = new TrajectoryBuilder(new Pose2d(-26, 30, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(49, 30), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(30, 30), () -> {
                    //Bring arm up
                    leftAutoArm.setPower(-0.1);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(45, 30), () -> {
                    //Rotate arm
                    leftAutoRotate.setPosition(0.16);
                    return Unit.INSTANCE;
                })
                .build();

        //Strafe into foundation
        Trajectory foundationFirstTimeSecondStrafe_CENTER = new TrajectoryBuilder(new Pose2d(49, 30, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(49, 14))
                .build();

        //Drive to Second Skystone (Wall) first strafe
        Trajectory secondSkystoneWallFirstStrafe_CENTER = new TrajectoryBuilder(new Pose2d(49, 14, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(49, 26))
                .build();

        //Drive to Second Skystone (Corner)
        Trajectory secondSkystoneCorner_CENTER = new TrajectoryBuilder(new Pose2d(49, 26, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(-20, 26), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(-10, 26), () -> {
                    //Rotate arm
                    leftFirstStoneArmPrep();
                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(-56, 63))
                .build();

        //Drive to Second Skystone (Block)
        Trajectory secondSkystone_CENTER = new TrajectoryBuilder(new Pose2d(-63, 63, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-51, 34))
                .strafeTo(new Vector2d(-51, 21))
                .build();

        //Drive to Foundation Long distance
        Trajectory foundationSecondTimeStraight_CENTER = new TrajectoryBuilder(new Pose2d(-51, 21, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(-9, 30), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .lineTo(new Vector2d(50, 30), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(6, 30), () -> {
                    //Lower arm slightly
                    leftAutoArm.setPower(0.60);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(41, 30), () -> {
                    //Rotate arm
                    leftAutoRotate.setPosition(0.16);
                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(50, -10))
                .build();

//--------------------
//    Foundation
//--------------------

        //Turn away from foundations (LineTo while Turning)
        Trajectory grabberToFoundation_CENTER = new TrajectoryBuilder(new Pose2d(51, 28, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(45, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                .lineTo(new Vector2d(45, 34), new SplineInterpolator(Math.toRadians(110), Math.toRadians(85)))
                .lineTo(new Vector2d(45, 24), new SplineInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build();

        //Move Foundation
        Trajectory moveFoundation_CENTER = new TrajectoryBuilder(new Pose2d(45, 23, Math.toRadians(90)), normalConstraint)
                .lineTo(new Vector2d(18, 33), new SplineInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build();

        //Push Foundation
        Trajectory pushFoundation_CENTER = new TrajectoryBuilder(new Pose2d(18, 33, Math.toRadians(190)), normalConstraint)
                .lineTo(new Vector2d(52, 63), new SplineInterpolator(Math.toRadians(190), Math.toRadians(180)))
                .build();

        //Park
        Trajectory park_CENTER = new TrajectoryBuilder(new Pose2d(50, 63, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(4, 58), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .build();

        /**
         * RIGHT
         */

        //Move to First Skystone
        Trajectory diagonalToFirstSkystone_RIGHT = new TrajectoryBuilder(new Pose2d(-33, 63, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-34, 27))
                .strafeTo(new Vector2d(-34, 20.5))
                .build();

        //Drive to Foundation first strafe (away from quarry)
        Trajectory foundationFirstTimeFirstStrafe_RIGHT = new TrajectoryBuilder(new Pose2d(-34, 20.5, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-34, 34))
                .build();

        //Straight-away to Foundation
        Trajectory foundationFirstTimeStraight_RIGHT = new TrajectoryBuilder(new Pose2d(-34, 34, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(48.5, 34), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(30, 34), () -> {
                    //Bring arm up
                    leftAutoArm.setPower(-0.1);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(40, 34), () -> {
                    //Rotate arm
                    leftAutoRotate.setPosition(0.16);
                    return Unit.INSTANCE;
                })
                .build();

        //Strafe into foundation
        Trajectory foundationFirstTimeSecondStrafe_RIGHT = new TrajectoryBuilder(new Pose2d(48.5, 34, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(48.5, 22))
                .build();

        //Drive to Second Skystone (Wall) first strafe
        Trajectory secondSkystoneWallFirstStrafe_RIGHT = new TrajectoryBuilder(new Pose2d(48.5, 22, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(48.5, 29))
                .build();

        //Drive to Second Skystone (Corner)
        Trajectory secondSkystoneCorner_RIGHT = new TrajectoryBuilder(new Pose2d(48.5, 29, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(-20, 29), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(-10, 29), () -> {
                    //Rotate arm
                    leftFirstStoneArmPrep();
                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(-59, 59))
                .build();

        //Drive to Second Skystone (Block)
        Trajectory secondSkystone_RIGHT = new TrajectoryBuilder(new Pose2d(-63, 63, Math.toRadians(180)), normalConstraint)
                .strafeTo(new Vector2d(-59, 45))
                .strafeTo(new Vector2d(-59, 21))
                .build();

        //Drive to Foundation Long distance
        Trajectory foundationSecondTimeStraight_RIGHT = new TrajectoryBuilder(new Pose2d(-59, 21, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(-9, 31), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .lineTo(new Vector2d(55, 31), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .addMarker(new Vector2d(-45, 31), () -> {
                    //Rotate around
                    leftAutoRotate.setPosition(0.8);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(6, 31), () -> {
                    //Lower arm slightly
                    leftAutoArm.setPower(0.60);
                    return Unit.INSTANCE;
                })
                .addMarker(new Vector2d(41, 31), () -> {
                    //Rotate arm
                    leftAutoRotate.setPosition(0.16);
                    return Unit.INSTANCE;
                })
                .strafeTo(new Vector2d(55, 5))
                .build();

//--------------------
//    Foundation
//--------------------

        //Turn away from foundations (LineTo while Turning)
        Trajectory grabberToFoundation_RIGHT = new TrajectoryBuilder(new Pose2d(51, 28, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(45, 40), new SplineInterpolator(Math.toRadians(180), Math.toRadians(90)))
                .lineTo(new Vector2d(45, 34), new SplineInterpolator(Math.toRadians(110), Math.toRadians(83)))
                .lineTo(new Vector2d(45, 23), new SplineInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build();

        //Move Foundation
        Trajectory moveFoundation_RIGHT = new TrajectoryBuilder(new Pose2d(45, 23, Math.toRadians(90)), normalConstraint)
                .lineTo(new Vector2d(18, 37), new SplineInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build();

        //Push Foundation
        Trajectory pushFoundation_RIGHT = new TrajectoryBuilder(new Pose2d(18, 37, Math.toRadians(190)), normalConstraint)
                .lineTo(new Vector2d(52, 67), new SplineInterpolator(Math.toRadians(190), Math.toRadians(180)))
                .build();

        //Park
        Trajectory park_RIGHT = new TrajectoryBuilder(new Pose2d(50, 72, Math.toRadians(180)), normalConstraint)
                .lineTo(new Vector2d(5, 60), new SplineInterpolator(Math.toRadians(180), Math.toRadians(180)))
                .build();
//      Runnable Code
        telemetry.update();


        if(opModeIsActive()){
            drive.followTrajectorySync(diagonalToFirstSkystone_LEFT);
        }
//
//        while (opModeIsActive()) {
//            switch (skystonePos) {
//
//                case 0:
//                    telemetry.update();
//                    skystonePos = 0;
//                    telemetry.addLine("Skystone is on the Left");
//                    telemetry.update();
//
//                    leftFirstStoneArmPrep();
//                    drive.followTrajectorySync(diagonalToFirstSkystone_LEFT);
//                    leftFirstSkystoneGrab();
//                    drive.followTrajectorySync(foundationFirstTimeFirstStrafe_LEFT);
//                    drive.followTrajectorySync(foundationFirstTimeStraight_LEFT);
//                    drive.followTrajectorySync(foundationFirstTimeSecondStrafe_LEFT);
//                    leftPlaceSkystoneOnFoundation();
//                    drive.followTrajectorySync(secondSkystoneWallFirstStrafe_LEFT);
//                    drive.followTrajectorySync(secondSkystoneCorner_LEFT);
//                    encoderDrive(0.5, 999, 0, 0, 999, 1.2);
//                    drive.followTrajectorySync(secondSkystone_LEFT);
//                    sleep(200);
//                    leftFirstSkystoneGrab();
//                    drive.followTrajectorySync(foundationSecondTimeStraight_LEFT);
//                    leftPlaceSkystoneOnFoundation();
//                    leftArmFullReset();
//
////                    drive.followTrajectorySync(grabberToFoundation_LEFT);
////                    grabFoundation();
////                    drive.followTrajectorySync(moveFoundation_LEFT);
////                    drive.turnSync(Math.toRadians(120));
////                    drive.followTrajectorySync(pushFoundation_LEFT);
////                    releaseFoundation();
////                    //Toggle Break
////                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    drive.followTrajectorySync(park_LEFT);
//
//                    sleep(4000);
//                    break;
//                case 1:
//                    telemetry.update();
//                    skystonePos = 1;
//                    telemetry.addLine("Skystone is on the Middle");
//                    telemetry.addData("Path: ", "Center");
//                    telemetry.update();
//
//                    leftFirstStoneArmPrep();
//                    drive.followTrajectorySync(diagonalToFirstSkystone_CENTER);
//                    leftFirstSkystoneGrab();
//                    drive.followTrajectorySync(foundationFirstTimeFirstStrafe_CENTER);
//                    drive.followTrajectorySync(foundationFirstTimeStraight_CENTER);
//                    drive.followTrajectorySync(foundationFirstTimeSecondStrafe_CENTER);
//                    leftPlaceSkystoneOnFoundation();
//                    drive.followTrajectorySync(secondSkystoneWallFirstStrafe_CENTER);
//                    drive.followTrajectorySync(secondSkystoneCorner_CENTER);
//                    encoderDrive(0.5, 999, 0, 0, 999, 1.2);
//                    drive.followTrajectorySync(secondSkystone_CENTER);
//                    sleep(200);
//                    leftFirstSkystoneGrab();
//                    drive.followTrajectorySync(foundationSecondTimeStraight_CENTER);
//                    leftPlaceSkystoneOnFoundation();
//                    leftArmFullReset();
//
////                    drive.followTrajectorySync(grabberToFoundation_CENTER);
////                    grabFoundation();
////                    drive.followTrajectorySync(moveFoundation_CENTER);
////                    drive.turnSync(Math.toRadians(120));
////                    drive.followTrajectorySync(pushFoundation_CENTER);
////                    releaseFoundation();
////                    //Toggle Break
////                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    drive.followTrajectorySync(park_CENTER);
//
//                    sleep(4000);
//                    break;
//                case 2:
//                    telemetry.update();
//                    skystonePos = 2;
//                    telemetry.addLine("Skystone is on the Right");
//                    telemetry.update();
//
//                    leftFirstStoneArmPrep();
//                    drive.followTrajectorySync(diagonalToFirstSkystone_RIGHT);
//                    leftFirstSkystoneGrab();
//                    drive.followTrajectorySync(foundationFirstTimeFirstStrafe_RIGHT);
//                    drive.followTrajectorySync(foundationFirstTimeStraight_RIGHT);
//                    drive.followTrajectorySync(foundationFirstTimeSecondStrafe_RIGHT);
//                    leftPlaceSkystoneOnFoundation();
//                    drive.followTrajectorySync(secondSkystoneWallFirstStrafe_RIGHT);
//                    drive.followTrajectorySync(secondSkystoneCorner_RIGHT);
//                    encoderDrive(0.5, 999, 0, 0, 999, 1.2);
//                    drive.followTrajectorySync(secondSkystone_RIGHT);
//                    sleep(200);
//                    leftSkystoneGrabNoRotate();
//                    drive.followTrajectorySync(foundationSecondTimeStraight_RIGHT);
//                    leftPlaceSkystoneOnFoundation();
//                    leftArmFullReset();
//
////                    drive.followTrajectorySync(grabberToFoundation_RIGHT);
////                    grabFoundation();
////                    drive.followTrajectorySync(moveFoundation_RIGHT);
////                    drive.turnSync(Math.toRadians(114));
////                    drive.followTrajectorySync(pushFoundation_RIGHT);
////                    releaseFoundation();
////                    //Toggle Break
////                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////                    drive.followTrajectorySync(park_RIGHT);
//
//                    sleep(4000);
//
//                    break;
//            }
//            break;
//        }
    }
/* ------------------------------------------------------------
          Down Below Are Methods Called in Our Runnable Code:
---------------------------------------------------------------*/

    /* ------------------------------
          OpenCV Main Pipeline
    ---------------------------------*/
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */
            int currentStageNum = stageToRenderToViewport.ordinal();
            int nextStageNum = currentStageNum + 1;
            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }
            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
//color diff cb.
//lower cb = more blue = skystone = white
//higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores
//b&w
//outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
//Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours;
//create rectangles to detect from
            submat1 = yCbCrChan2Mat.submat(new Rect(pointA, pointB));
            submat2 = yCbCrChan2Mat.submat(new Rect(pointC, pointD));
            submat3 = yCbCrChan2Mat.submat(new Rect(pointE, pointF));


            // take the average value over each rectangle
            avg1 = (int) Core.mean(submat1).val[0];
            avg2 = (int) Core.mean(submat2).val[0];
            avg3 = (int) Core.mean(submat3).val[0];


            // Figure out which sample zone had the lowest contrast from blue (lightest color)
            max = Math.max(avg1, Math.max(avg2, avg3));
            if (max == avg1) {
                skystonePos = 0;
            } else if (max == avg2) {
                skystonePos = 1;
            } else if (max == avg3) {
                skystonePos = 2;
            }


            Imgproc.rectangle(
                    all,                    //Matrix obj of the image
                    new Point(215, 155),        //p1
                    new Point(225, 145),       //p2
                    new Scalar(0, 0, 255),     //Scalar object for color
                    5                          //Thickness of the line
            );


            Imgproc.rectangle(
                    all,                    //Matrix obj of the image
                    new Point(361, 155),        //p1
                    new Point(371, 145),       //p2
                    new Scalar(0, 0, 255),     //Scalar object for color
                    5                          //Thickness of the line
            );

            Imgproc.rectangle(
                    all,                    //Matrix obj of the image
                    new Point(510, 155),        //p1
                    new Point(520, 145),       //p2
                    new Scalar(0, 0, 255),     //Scalar object for color
                    5                          //Thickness of the line
            );
            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }
                case detection: {
                    return all;
                }
                case RAW_IMAGE: {
                    return input;
                }
                default: {
                    return input;
                }
            }
        }


    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double calculatedSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            //Set target positions
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //Send power to motors
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (
                    (runtime.seconds() < timeoutS) &&
                            (leftFront.isBusy() || rightFront.isBusy() || rightBack.isBusy() || leftBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            //Reset ticks
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    //----------------------------------------------------------------------------------------------
    // Methods for Foundation
    //----------------------------------------------------------------------------------------------

    //Method for Grabbing Foundation
    public void grabFoundation() {
        leftFoundation.setPosition(0.3);
        rightFoundation.setPosition(0.6);
        sleep(300);
    }

    //Method for Releasing Foundation
    public void releaseFoundation() {
        leftFoundation.setPosition(0.85);
        rightFoundation.setPosition(0.1);
        sleep(350);
    }

    public void leftFirstStoneArmPrep() {
        leftAutoArm.setPower(0.9);
        leftAutoGripper.setPosition(0.4);
        leftAutoRotate.setPosition(0.16);
    }

    public void leftFirstStoneArmPrepNoRotate() {
        leftAutoArm.setPower(0.9);
        leftAutoGripper.setPosition(0.4);
    }

    public void rightFirstStoneArmprep() {
        rightAutoArm.setPower(-0.44);
        rightAutoGripper.setPosition(0.25);
        rightAutoRotate.setPosition(0.7);
    }

    public void leftFirstSkystoneGrab() {
        //Grab
        leftAutoGripper.setPosition(0);
        runtime.reset();
        while ((runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm up
        leftAutoArm.setPower(0.5);
        runtime.reset();
        while ((runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Rotate around
        leftAutoRotate.setPosition(0.8);
    }

    public void rightFirstSkystoneGrab() {
        //Grab
        rightAutoGripper.setPosition(0.7);
        runtime.reset();
        while ((runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm up
        rightAutoArm.setPower(0);
        runtime.reset();
        while ((runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Rotate around
        rightAutoRotate.setPosition(0);
    }

    public void leftSkystoneGrabNoRotate() {
        //Grab
        leftAutoGripper.setPosition(0);
        runtime.reset();
        while ((runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm up
        leftAutoArm.setPower(0.54);
    }

    public void rightSkystoneGrabNoRotate() {
        //Grab
        rightAutoGripper.setPosition(0.7);
        runtime.reset();
        while ((runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm up
        rightAutoArm.setPower(0);
    }

    public void leftFrontGrab() {
        //Grab Block
        //Arm down, Gripper Open
        leftAutoArm.setPower(1);
        leftAutoGripper.setPosition(0.1);
        runtime.reset();
        while ((runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Grab
        leftAutoGripper.setPosition(0.4);
        runtime.reset();
        while ((runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Arm up
        leftAutoArm.setPower(0.52);
    }

    public void leftPlaceSkystoneOnFoundation() {
        //Bring arm down
        leftAutoArm.setPower(0.8);
        runtime.reset();
        while ((runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Release block
        leftAutoGripper.setPosition(0.4);
        runtime.reset();
        while ((runtime.seconds() < 0.35)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Bring arm back up
        leftAutoArm.setPower(0.5);
        runtime.reset();
        while ((runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Reset Arm
        leftAutoArm.setPower(0.9);
        leftAutoRotate.setPosition(0.65);
    }

    public void rightPlaceSKystoneOnFoundation() {
        //Bring arm down
        rightAutoArm.setPower(-0.3);
        runtime.reset();
        while ((runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Release block
        rightAutoGripper.setPosition(0.25);
        runtime.reset();
        while ((runtime.seconds() < 0.35)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Bring arm back up
        rightAutoArm.setPower(0.5);
        runtime.reset();
        while ((runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Reset Arm
        rightAutoArm.setPower(-0.4);
        rightAutoRotate.setPosition(0.2);
    }

    public void leftReleaseSkystone() {
        //Release block
        leftAutoGripper.setPosition(0.1);
        runtime.reset();
        while ((runtime.seconds() < 0.33)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Bring arm back up
        leftAutoArm.setPower(0);
        runtime.reset();
        while ((runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Reset Arm
        leftAutoArm.setPower(0.7);
        leftAutoRotate.setPosition(0.8);
    }

    public void leftArmFullReset() {
        leftAutoArm.setPower(-1);
        leftAutoGripper.setPosition(0);
        leftAutoRotate.setPosition(0.65);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    //
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
