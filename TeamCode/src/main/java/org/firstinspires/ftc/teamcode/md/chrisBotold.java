package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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
public class chrisBotold
{
    private String version = "1.1";

    /** MOTOR OBJECTS */
    public DcMotor  motorBackLeft   = null, motorFrontLeft  = null, motorFrontRight  = null, motorBackRight  = null;
    public DcMotorSimple motorIntake = null, motorShooter1 = null, motorShooter2 = null, motorBottomIntake = null;

    public WebcamName webcam = null;

    public ColorSensor colorL = null, colorR = null;

    public static final double COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION    = (double)2/(double)3 ;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES   = 2.95276 ;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = chrisBotConstants.DRIVE_SPEED, TURN_SPEED = chrisBotConstants.TURN_SPEED, JIGGLE_SPEED = chrisBotConstants.JIGGLE_SPEED;

    int FLTarget = 0, FRTarget = 0, BLTarget = 0, BRTarget = 0;

    public static final double shootPower = chrisBotConstants.shootPower, shootPowerSlow = chrisBotConstants.shootPowerSlow;

    public boolean shooterOn = chrisBotConstants.shooterOn, intakeOn = chrisBotConstants.intakeOn;


    /** GYRO OBJECTS */

    public BNO055IMU imu = null;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();
    double globalAngle;

    /** VUFORIA OBJECTS */

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final float mmPerInch = 25.4f, mmTargetHeight = (6) * mmPerInch;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch, quadField  = 36 * mmPerInch;

    // private boolean targetVisible = false;
    private float phoneXRotate = 0, phoneYRotate = 0, phoneZRotate = 0;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private OpenGLMatrix lastLocation = null;

    public VuforiaLocalizer vuforia;

    public TFObjectDetector tfod;

    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite", LABEL_FIRST_ELEMENT = "Quad", LABEL_SECOND_ELEMENT = "Single";

    public static final String VUFORIA_KEY = chrisBotConstants.key1;

    /** local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    private Telemetry telemetry;

    public boolean shooterExists = false, intakeExists = false, webcamExists = false, intakeBottomExists = false, colorExists = false;

    /* Constructor */
    public chrisBotold() { }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        if (this.telemetry == null) {
            this.telemetry = telemetry;
            this.telemetry.setAutoClear(false);
            telemetry.addLine("Booting...");
            telemetry.update();
        }

        // Save reference to Hardware map
        hwMap = ahwMap;

        /** Vuforia section */

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        /** Drive motor section */

        // Define and Initialize Motors

        motorBackLeft  = hwMap.get(DcMotor.class, "motorBackLeft");
        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");

        motorBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBackRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run with encoders.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        /** Attachment section */

        intakeExists = hwMap.tryGet(DcMotorSimple.class, "motorIntake") != null;
        intakeBottomExists = hwMap.tryGet(DcMotorSimple.class, "motorBottomIntake") != null;
        shooterExists = hwMap.tryGet(DcMotorSimple.class, "motorShooter1") != null && hwMap.tryGet(DcMotorSimple.class, "motorShooter2") != null;
        webcamExists = hwMap.tryGet(WebcamName.class, "Webcam 1") != null;
        colorExists = hwMap.tryGet(ColorSensor.class, "colorL") != null && hwMap.tryGet(ColorSensor.class, "colorR") != null;

        if (intakeExists) {
            motorIntake = hwMap.get(DcMotorSimple.class, "motorIntake");
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            motorIntake.setPower(0);

            telemetry.addLine("Intake motor initialized");
            telemetry.update();
        }

        if (intakeBottomExists) {
            motorBottomIntake = hwMap.get(DcMotorSimple.class, "motorBottomIntake");
            motorBottomIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBottomIntake.setPower(0);

            telemetry.addLine("Bottom Intake motor initialized");
            telemetry.update();
        }

        if (shooterExists) {
            motorShooter1 = hwMap.get(DcMotorSimple.class, "motorShooter1");
            motorShooter1.setDirection(DcMotorSimple.Direction.FORWARD);
            motorShooter1.setPower(0);

            motorShooter2 = hwMap.get(DcMotorSimple.class, "motorShooter2");
            motorShooter2.setDirection(DcMotorSimple.Direction.REVERSE);
            motorShooter2.setPower(0);

            telemetry.addLine("Shooter motor initialized");
            telemetry.update();
        }

        if (webcamExists) {
            webcam = hwMap.get(WebcamName.class, "Webcam 1");

            telemetry.addLine("Webcam initialized");
            telemetry.update();
        }

        if (colorExists) {
            colorL = hwMap.get(ColorSensor.class, "colorL");
            colorR = hwMap.get(ColorSensor.class, "colorR");

            telemetry.addLine("Color sensors initialized");
            telemetry.update();
        }

        setAllDrivePower(0);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 1000);

        welcome();

    }

    public void init(HardwareMap hwMap, Telemetry telemetry, boolean initVuforia, boolean initTfod) {
        this.telemetry = telemetry;
        this.telemetry.setAutoClear(false);

        telemetry.addLine("Booting...");
        telemetry.update();

        if (initVuforia) {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.

            telemetry.addLine("Vuforia initialized successfully");
            telemetry.update();
        }
        if (initTfod) {
            int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

            telemetry.addLine("TensorFlow OD initialized successfully");
            telemetry.update();
        }

        init(hwMap, telemetry);
    }

    /** MOTOR METHODS */
    // This method returns true if any drive motor is busy and false otherwise.
    public boolean isBusy() {
        return motorFrontLeft.isBusy() || motorBackRight.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy();
    }
    // This method sets the powers of all the drive motors on the robot.
    public void setAllDrivePower(double speed) {
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);
    }
    // This method sets the encoder drive targets on the actual motors.
    public void setDriveTargets() {
        motorFrontLeft.setTargetPosition(FLTarget);
        motorFrontRight.setTargetPosition(FRTarget);
        motorBackLeft.setTargetPosition(BLTarget);
        motorBackRight.setTargetPosition(BRTarget);
    }
    // This method sets the modes of all the motors.
    public void setAllDriveMode(DcMotor.RunMode mode) {
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

    // This method checks power levels to be safe and pushes them to the motors
    public void setDrivePower(double flPower, double frPower, double blPower, double brPower) {
        setDrivePower(new double[]{flPower, frPower, blPower, brPower});
    }
    public void setDrivePower(double[] powers) {
        // Check deadzones
        for (int i = 0; i < powers.length; i++) {
            powers[i] = Range.clip(powers[i], -1, 1);
        }
        // Push powers
        motorFrontLeft.setPower(powers[0]);
        motorFrontRight.setPower(powers[1]);
        motorBackLeft.setPower(powers[2]);
        motorBackRight.setPower(powers[3]);
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

        setDriveTargets();

        // Turn On RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        setAllDrivePower(DRIVE_SPEED);

        // keep looping while we are still active, and there is time left and motors are running.
        while (isBusy()) {
        }

        // Stop all motion;
        setAllDrivePower(0);

        // Turn off RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double inches) {
        encoderDrive(DRIVE_SPEED, inches);
    }

    public void byWheelEncoderDrive(double flInches, double frInches, double blInches, double brInches, double speed) {
        resetTargets();

        // Determine new target position, and pass to motor controller
        FLTarget = motorFrontLeft.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
        FRTarget = motorFrontRight.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
        BLTarget = motorBackLeft.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
        BRTarget = motorBackRight.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);

        setDriveTargets();

        // Turn On RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        setAllDrivePower(speed);

        // keep looping while we are still active, and there is time left and motors are running.
        while (isBusy()) {
        }

        // Stop all motion;
        setAllDrivePower(0);

        // Turn off RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void byWheelEncoderDrive(double flInches, double frInches, double blInches, double brInches) {
        byWheelEncoderDrive(flInches, frInches, blInches, brInches, DRIVE_SPEED);
    }

    // This overloaded method allows the robot to drive with encoders on a per wheel basis. It can be called with inches and with or without a drive speed.
    // This method can be used to strafe if used in combination with calculateInches().
    public void wheelMecanumDrive(double[] inches, double speed, double timeoutS) {
        resetTargets();

        // Calculate the maximum number of inches any wheel is asked to drive
        double inchesMax = 0;
        for (double inch : inches) {
            if (inch > inchesMax) {
                inchesMax = inch;
            }
        }

        // Determine new target position, and pass to motor controller
        FLTarget = motorFrontLeft.getCurrentPosition() + (int) (inches[0] * COUNTS_PER_INCH);
        FRTarget = motorFrontRight.getCurrentPosition() + (int) (inches[1] * COUNTS_PER_INCH);
        BLTarget = motorBackLeft.getCurrentPosition() + (int) (inches[2] * COUNTS_PER_INCH);
        BRTarget = motorBackRight.getCurrentPosition() + (int) (inches[3] * COUNTS_PER_INCH);

        setDriveTargets();

        // Turn On RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        setAllDrivePower(DRIVE_SPEED);

        // keep looping while we are still active, and there is time left and motors are running.
        while ((runtime.seconds() < timeoutS) && isBusy()) {
        }

        // Stop all motion;
        setAllDrivePower(0);

        // Turn off RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void wheelMecanumDrive(double[] inches) {
        wheelMecanumDrive(inches, DRIVE_SPEED, 9999);
    }

    public void wheelMecanumDrive(double[] inches, double speed) { wheelMecanumDrive(inches, speed, 9999);
    }

    public void linearSlowEncoderDrive(double inches, double speed) {
        double[] powers = {speed, speed, speed, speed};
        int[] motorPositionsOld = {motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition()};
        int[] motorPositions = motorPositionsOld;
        resetTargets();

        // Determine new target position, and pass to motor controller
        int countsToTravel = (int)(inches * COUNTS_PER_INCH);
        FLTarget = motorFrontLeft.getCurrentPosition() + countsToTravel;
        FRTarget = motorFrontRight.getCurrentPosition() + countsToTravel;
        BLTarget = motorBackLeft.getCurrentPosition() + countsToTravel;
        BRTarget = motorBackRight.getCurrentPosition() + countsToTravel;
        int[] targets = {FLTarget, FRTarget, BLTarget, BRTarget};

        setDriveTargets();

        // Turn On RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        setDrivePower(powers);

        Telemetry.Item a = telemetry.addData("fl",powers[0]);
        Telemetry.Item b = telemetry.addData("fr",powers[1]);
        Telemetry.Item c = telemetry.addData("bl",powers[2]);
        Telemetry.Item d = telemetry.addData("br",powers[3]);
        telemetry.update();

        // keep looping while we are still active, and there is time left and motors are running.
        while (isBusy()) {
            for(int i : range(0,4)) {
                motorPositions = new int[]{motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition()};
                double power = powers[i]*(1 - (double)(motorPositions[i]-motorPositionsOld[i])/(double)countsToTravel);
                if (power < 0.5) {
                    power = 0.5;
                }
                powers[i] = power;
            }
            setDrivePower(powers);
            a.setValue(powers[0]);
            b.setValue(powers[1]);
            c.setValue(powers[2]);
            d.setValue(powers[3]);
            telemetry.update();
        }

        // Stop all motion;
        setAllDrivePower(0);

        // Turn off RUN_TO_POSITION
        setAllDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*public void halfGyroDrive(double inches) { halfGyroDrive(inches, DRIVE_SPEED); }

    public void halfGyroDrive(double inches, double speed) {
        double[] powers = {speed, speed, speed, speed};
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
        setPower(powers);

        double angle = getOrientation().firstAngle;
        PID angleCorrector = new PID(angle, new double[]{0.02,0.02,0});
        double correction = 0;

        // keep looping while we are still active, and there is time left and motors are running.
        while (isBusy()) {
            angleCorrector.actual = getOrientation().firstAngle;
            correction = angleCorrector.calcCorrection();
            powers[0] = powers[0] + correction;
            powers[1] = powers[1] - correction;
            powers[2] = powers[2] + correction;
            powers[3] = powers[3] - correction;
            setPower(powers);
        }

        // Stop all motion;
        setAllPower(0);

        // Turn off RUN_TO_POSITION
        setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/
    /*
    public void stemPIDdrive(int ms, double power) {
        PIDController pidDrive = new PIDController(chrisBotConstants.drive_Kp, chrisBotConstants.drive_Ki, chrisBotConstants.drive_Kd);
        ElapsedTime e = new ElapsedTime();
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // drive until end of period.
        e.reset();
        Telemetry.Item a = telemetry.addData("1 imu heading", lastAngles.firstAngle);
        Telemetry.Item b = telemetry.addData("2 global heading", globalAngle);
        Telemetry.Item c = telemetry.addData("3 correction", 0);
        telemetry.update();
        while (e.milliseconds() < ms)
        {
            // Use PID with imu input to drive in a straight line.
            double correction = pidDrive.performPID(getAngle());

            a.setValue(lastAngles.firstAngle);
            b.setValue(globalAngle);
            c.setValue(correction);
            telemetry.update();

            // set power levels.
            setDrivePower(power + correction, power - correction, power + correction, power - correction);

        }

        // turn the motors off.
        setAllDrivePower(0);
    }

     */

    public void lineUp() {
        boolean Ltripped = false, Rtripped = false;
        boolean rightFirst = false;
        setAllDrivePower(0.1);
        while(true) {
            if(Ltripped && Rtripped) {
                break;
            } else if (Ltripped) {
                motorBackLeft.setPower(-0.06);
                motorFrontLeft.setPower(-0.06);
                motorFrontRight.setPower(0.1);
                motorBackRight.setPower(0.1);
                if(white(colorR)) {
                    Rtripped = true;
                }
            } else if (Rtripped) {
                motorBackRight.setPower(-0.06);
                motorFrontRight.setPower(-0.06);
                motorFrontLeft.setPower(0.1);
                motorBackLeft.setPower(0.1);
                if(white(colorL)) {
                    Ltripped = true;
                }
            } else {
                setAllDrivePower(0.1);
                if(white(colorL)) {
                    Ltripped = true;
                } else if(white(colorR)) {
                    Rtripped = true;
                }
            }
        }
        setAllDrivePower(0);
    }

    /** ATTACHMENT METHODS */
    public void autonShoot() {
        lineUp();

        busyWait(500);

        byWheelEncoderDrive(0,-3,0,-3,0.25);
        busyWait(2000);

        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < 200) {
            intakeOn();
        }
        intakeOff();
        shootOn(0.237);
        byWheelEncoderDrive(0,-2,0,-2,0.1);
        busyWait(6000);
        e.reset();
        while(e.milliseconds() < 230) {
            intakeOn();
        }
        intakeOff();
        shootOn(0.24);
        byWheelEncoderDrive(0,-1.5,0,-1.5,0.1);
        busyWait(3000);
        e.reset();
        while(e.milliseconds() < 700) {
            intakeOn();
        }
        intakeOff();
    }

    // This method runs the motors in order to drop the Wobble Goal.
    public void dropGoal() {
        /* Code to drop goal goes here */
        telemetry.addLine("Wobble goal dropped");
        telemetry.update();
    }

    public void shootOnSlow() {
        if(shooterExists) {
            motorShooter1.setPower(shootPowerSlow);
            motorShooter2.setPower(shootPowerSlow);
            shooterOn = true;
        }
    }
    public void shootOn(double x) {
        if(shooterExists) {
            motorShooter1.setPower(x);
            motorShooter2.setPower(x);
            shooterOn = true;
        }
    }
    public void shootOn() {
        if(shooterExists) {
            motorShooter1.setPower(shootPower);
            motorShooter2.setPower(shootPower);
            shooterOn = true;
        }
    }
    public void shootOff() {
        if(shooterExists) {
            motorShooter1.setPower(0);
            motorShooter2.setPower(0);
            shooterOn = false;
        }
    }
    public void shootReverse() {
        if(shooterExists) {
            motorShooter1.setPower(-1);
            motorShooter2.setPower(-1);
            shooterOn = true;
        }
    }
    // These methods turn the intake motor on and off, at a set power or at full power.
    public void intakeOn() {
        if(intakeExists) {
            motorIntake.setPower(1);
            motorBottomIntake.setPower(1);
            intakeOn = true;
        }
    }
    public void intakeBottomReverse() {
        if(intakeBottomExists) {
            motorBottomIntake.setPower(-1);
            intakeOn = true;
        }
    }
    public void intakeBottom() {
        if(intakeBottomExists) {
            motorBottomIntake.setPower(1);
            intakeOn = true;
        }
    }
    public void intakeTop() {
        if(intakeExists) {
            motorIntake.setPower(1);
            intakeOn = true;
        }
    }
    public void intakeOff() {
        if(intakeExists) {
            motorIntake.setPower(0);
            motorBottomIntake.setPower(0);
            intakeOn = false;
        }
    }

    /** SENSOR METHODS */
    public Orientation getOrientation() {
        // Code to get the gyro orientation goes here
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    // stemrobotics code
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    public double getAngle() {
        Orientation x = getOrientation();
        double deltaAngle = (double)x.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public boolean white(ColorSensor c) {
        int[] color = {c.red(), c.green(), c.blue()};
        for(int i = 0; i < 3; i++) {
            if(color[i]<chrisBotConstants.white[i]) {
                return false;
            }
        }
        return true;
    }

    /** VUFORIA METHODS */

    /*

    // This detects the number of rings in front of the robot (for use in auton). Vuforia and TFOD must be initialized first.
    public boolean[] detectRings() {

        telemetry.addLine("Detecting rings... (those pesky orange toruses)");
        telemetry.update();

        ArrayList<ringObject> rings = ringObject.detectRings(tfod);

        boolean is1ring = false;
        boolean is4rings = false;

        for (ringObject ring : rings) {
            is1ring = (ring.label.equals("Single"));
            is4rings = (ring.label.equals("Quad"));
        }

        telemetry.addData("One ring detected",is1ring);
        telemetry.addData("Four rings detected",is4rings);

        return new boolean[]{is1ring, is4rings};
    }
    public Position detectTargets() {
        // check all the trackable targets to see which one (if any) is visible.
        String name = "";
        Position P;
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                name = trackable.getName();
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            // Send to variable
            P = new Position(name, translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch,
                    rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            P = null;
        }
        return P;
    }

     */

    /** MATH/CALCULATION METHODS */
    // This method calculates the inches each mecanum wheel should turn to make the robot drive a certain number of x/y inches overall.
    // Positive x is to the right; Positive y is forward.
    public double[] calculateInches(double xInches, double yInches) {
        double r = Math.hypot(xInches, yInches);
        double robotAngle = Math.atan2(yInches, xInches) - Math.PI / 4;
        return new double[]{r * Math.cos(robotAngle), r * Math.sin(robotAngle), r * Math.sin(robotAngle), r * Math.cos(robotAngle)}; //fl,fr,bl,br
    }
    public static int[] range(int start, int end) {
        if (end - start <= 0) {
            return new int[]{};
        }
        else {
            int[] arr = new int[end - start];
            for(int i = start; i < end; i++) {
                arr[i-start] = i;
            }
            return arr;
        }
    }

    /** TELEMETRY METHODS */
    public void breakTelemetry() {
        telemetry.addLine("**************************************");
    }
    private void welcome() {
        breakTelemetry();
        telemetry.addLine("Welcome to ChrisBot version "+version+"!\nPlease wait a few seconds for the encoders to reset, so that Chris doesn't complain about not having gyro yet\nWhen you're ready to pwn some n00bs press the \"Play\" button");
        telemetry.update();
    }

    /** MISC METHODS */

    private void busyWait(long ms) {
        ElapsedTime e = new ElapsedTime();
        e.reset();
        while(e.milliseconds() < ms) {
            int i = (int)3;
        }
    }
}

