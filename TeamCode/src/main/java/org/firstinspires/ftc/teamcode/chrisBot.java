package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * This class builds on the concept of a hardware class by containing common methods (mostly involving motor action) so that they can be used without rewriting them in
 * new code or using lengthy class calls.
 *
 * This class assumes the following device names have been configured on the robot:
 *
 * four motors for Mecanum Drive named motorBackLeft, motorFrontLeft, motorFrontRight, motorBackRight
 * the internal hub IMU named imu
 * a webcam attached to USB.
 */
public class chrisBot
{
    /** MOTOR OBJECTS */
    public DcMotor  motorBackLeft   = null;
    public DcMotor  motorFrontLeft  = null;
    public DcMotor  motorFrontRight  = null;
    public DcMotor  motorBackRight  = null;

    public static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION    = 0.625 ;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES   = 2.95276 ;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.7;
    public static final double TURN_SPEED = 0.3;

    int FLTarget = 0;
    int FRTarget = 0;
    int BLTarget = 0;
    int BRTarget = 0;

    /** GYRO OBJECTS */

    public BNO055IMU imu = null;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /** VUFORIA OBJECTS */

    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";

    public static final String VUFORIA_KEY = vuforia_key.key1;

    /** local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public chrisBot(){ /* Nothing is done during construction so this is empty */ }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Define and Initialize Motors
        motorBackLeft  = hwMap.get(DcMotor.class, "motorBackLeft");
        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorBackRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors


        // Set all motors to zero power
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /** MOTOR METHODS */
    // This method returns true if any drive motor is busy and false otherwise.
    public boolean isBusy() {
        return motorFrontLeft.isBusy() || motorBackRight.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy();
    }
    // This method sets the powers of all the drive motors on the robot.
    public void setAllPower(double speed) {
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);
    }
    // This method sets the encoder drive targets on the actual motors.
    public void setTargets() {
        motorFrontLeft.setTargetPosition(FLTarget);
        motorFrontRight.setTargetPosition(FRTarget);
        motorBackLeft.setTargetPosition(BLTarget);
        motorBackRight.setTargetPosition(BRTarget);
    }
    // This method sets the modes of all the motors.
    public void setAllMode(DcMotor.RunMode mode) {
        motorFrontLeft.setMode(mode);
        motorFrontRight.setMode(mode);
        motorBackLeft.setMode(mode);
        motorBackRight.setMode(mode);
    }
    // This method resets the encoder driving targets.
    private void resetTargets() {
        FLTarget = 0;
        FRTarget = 0;
        BLTarget = 0;
        BRTarget = 0;
    }

    /** DRIVING METHODS */

    // This overloaded method allows the robot to drive back and forward. It can be called with inches and with or without a drive speed.
    public void encoderDrive(double speed, double inches) {
        resetTargets();

        // Determine new target position, and pass to motor controller
        int countsToTravel = (int)(inches * COUNTS_PER_INCH);
        FLTarget = motorFrontLeft.getCurrentPosition() + countsToTravel;
        FRTarget = motorFrontRight.getCurrentPosition() + countsToTravel;
        BLTarget = motorBackLeft.getCurrentPosition() + countsToTravel;
        BRTarget = motorBackRight.getCurrentPosition() + countsToTravel;

        setTargets();

        // Turn On RUN_TO_POSITION
        setAllMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        setAllPower(testPlatformHardware.DRIVE_SPEED);

        // keep looping while we are still active, and there is time left and motors are running.
        while (isBusy()) {
        }

        // Stop all motion;
        setAllPower(0);

        // Turn off RUN_TO_POSITION
        setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double inches) {
        encoderDrive(DRIVE_SPEED, inches);
    }

    // This overloaded method allows the robot to drive with encoders on a per wheel basis. It can be called with inches and with or without a drive speed.
    // This method can be used to strafe if used in combination with calculateInches().
    public void wheelMecanumDrive(double[] inches, double timeoutS) {
        resetTargets();

        // Calculate the maximum number of inches any wheel is asked to drive
        double inchesMax = 0;
        for (double inch : inches) {
            if (inch > inchesMax) {
                inchesMax = inch;
            }
        }

        // Determine new target position, and pass to motor controller
        FLTarget = motorFrontLeft.getCurrentPosition() + (int) (inches[0] * testPlatformHardware.COUNTS_PER_INCH);
        FRTarget = motorFrontRight.getCurrentPosition() + (int) (inches[1] * testPlatformHardware.COUNTS_PER_INCH);
        BLTarget = motorBackLeft.getCurrentPosition() + (int) (inches[2] * testPlatformHardware.COUNTS_PER_INCH);
        BRTarget = motorBackRight.getCurrentPosition() + (int) (inches[3] * testPlatformHardware.COUNTS_PER_INCH);

        setTargets();

        // Turn On RUN_TO_POSITION
        setAllMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        setAllPower(testPlatformHardware.DRIVE_SPEED);

        // keep looping while we are still active, and there is time left and motors are running.
        while ((runtime.seconds() < timeoutS) && isBusy()) {
        }

        // Stop all motion;
        setAllPower(0);

        // Turn off RUN_TO_POSITION
        setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void wheelMecanumDrive(double[] inches) {
        wheelMecanumDrive(inches, DRIVE_SPEED);
    }

    /** ATTACHMENT METHODS */

    // This method runs the motors in order to drop the Wobble Goal.
    public void dropGoal() {
        /* Code to drop goal goes here */
    }

    // This code runs the motors to shoot exactly one ring.
    public void shoot() {
        /* Code to shoot a ring goes here */
    }

    /** SENSOR METHODS */


    /** VUFORIA METHODS */

    // This initializes Vuforia for use.
    public void initVuforia() {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = chrisBot.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

    }

    // This initializes the TensorFlow object detection (TFOD) engine.
    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(chrisBot.TFOD_MODEL_ASSET, chrisBot.LABEL_FIRST_ELEMENT, chrisBot.LABEL_SECOND_ELEMENT);
    }

    // This detects the number of rings in front of the robot (for use in auton). Vuforia and TFOD must be initialized first.
    public boolean[] detectRings() {
        ArrayList<ringObject> rings = ringObject.detectRings(tfod);

        boolean is1ring = false;
        boolean is4rings = false;

        for (ringObject ring : rings) {
            is1ring = (ring.label.equals("Single"));
            is4rings = (ring.label.equals("Quad"));
        }

        return new boolean[]{is1ring, is4rings};
    }

    /** MATH/CALCULATION METHODS */
    // This method calculates the inches each mecanum wheel should turn to make the robot drive a certain number of x/y inches overall.
    // Positive x is to the right; Positive y is forward.
    public double[] calculateInches(double xInches, double yInches) {
        double r = Math.hypot(xInches, yInches);
        double robotAngle = Math.atan2(yInches, xInches) - Math.PI / 4;
        return new double[]{r * Math.cos(robotAngle), r * Math.sin(robotAngle), r * Math.sin(robotAngle), r * Math.cos(robotAngle)}; //fl,fr,bl,br
    }


}

