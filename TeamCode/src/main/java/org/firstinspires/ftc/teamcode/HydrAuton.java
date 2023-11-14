package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "HydrAuton", preselectTeleOp = "HyDrive")
public class HydrAuton extends LinearOpMode {

    private IMU imu;
    private DcMotor MotLwrArm;
    private DcMotor MotUprArm;
    private HydraDrive Drive;
    private Servo SrvPxlPos1;
    private Servo SrvPxlPos2;
    private ColorSensor SenColPxlPos1;
    private ColorSensor SenColPxlPos2;
    private HydraIntake Intake;
    private LED LED4;
    private LED LED3;
    private LED LED2;
    private LED LED1;
    private DistanceSensor SenColPxlPos1_DistanceSensor;
    private DistanceSensor SenColPxlPos2_DistanceSensor;

    int armPositionState;
    ElapsedTime pixelDropTimer;
    TfodProcessor myTfodProcessor;
    boolean armMoveToHome;
    boolean armMoveToBack;
    boolean armMoveToFront;
    boolean armMoveToHang;
    int autonState;
    int cObjectLocationUnknown;
    int foo;
    double cLowerArmAutoMotorPwr;
    String modelFilename;
    double cUpperArmAutoMotorPwr;
    int cPixelPos1Dist;
    int cPixelPos2Dist;
    int cPixelFrontScoreRunTimeMs;
    double cCountsPerInch;
    List cLowerArmPositions;
    boolean USE_WEBCAM;
    VisionPortal myVisionPortal;
    double cCasBackToFront;
    double cCasStop;
    double cCasFrontToBack;
    List cUpperArmPositions;
    int cPixelDropRunTimeMs;
    List cArmPositionNames;
    int objectLocation;
    int cObjectLocationLeft;
    int cObjectLocationCenter;
    int cObjectLocationRight;
    double cDriveNormal;
    double cDriveSlow;
    int cXvalueForLeftToCenterObject;
    private long pixelDropTimeStart;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double cWheelDiameter;
        double cWheelCircumference;
        double cCountsPerWheelRevolution;
        int cDriveBoosted;
        int cLowerArmPos0Home;
        int cUpperArmPos0Home;
        int cLowerArmPos1LiftBox;
        int cUpperArmPos1LiftBox;
        int cLowerArmPos2LiftArm;
        int cUpperArmPos2LiftArm;
        int cLowerArmPos3BackScore;
        int cUpperArmPos3BackScore;
        int cLowerArmPos4FrontScore;
        int cUpperArmPos4FrontScore;
        int cLowerArmPos5Hang;
        int cUpperArmPos5Hang;
        int cIntakeIn;
        int cIntakeOut;
        int cMaxObjectSearchTimeMs;
        ElapsedTime opModeTimer;
        boolean autonAbort;

        imu = hardwareMap.get(IMU.class, "imu");
        MotLwrArm = hardwareMap.get(DcMotor.class, "MotLwrArm");
        MotUprArm = hardwareMap.get(DcMotor.class, "MotUprArm");
        SrvPxlPos1 = hardwareMap.get(Servo.class, "SrvPxlPos1");
        SrvPxlPos2 = hardwareMap.get(Servo.class, "SrvPxlPos2");
        SenColPxlPos1 = hardwareMap.get(ColorSensor.class, "SenColPxlPos1");
        SenColPxlPos2 = hardwareMap.get(ColorSensor.class, "SenColPxlPos2");
        LED4 = hardwareMap.get(LED.class, "LED4");
        LED3 = hardwareMap.get(LED.class, "LED3");
        LED2 = hardwareMap.get(LED.class, "LED2");
        LED1 = hardwareMap.get(LED.class, "LED1");
        SenColPxlPos1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "SenColPxlPos1");
        SenColPxlPos2_DistanceSensor = hardwareMap.get(DistanceSensor.class, "SenColPxlPos2");

        // Initialize Constant Variables
        // Wheel constants
        cWheelDiameter = 3.78;
        cWheelCircumference = cWheelDiameter * Math.PI;
        cCountsPerWheelRevolution = 537.6;
        cCountsPerInch = cCountsPerWheelRevolution / cWheelCircumference;
        // Drive motor power level scaling [max 1]
        cDriveBoosted = 1;
        cDriveNormal = 0.9;
        cDriveSlow = 0.5;
        // Arm motor power scaling
        cLowerArmAutoMotorPwr = 0.5;
        cUpperArmAutoMotorPwr = 0.4;
        // Arm position constants
        armPositionState = 0;
        //
        cLowerArmPos0Home = 0;
        cUpperArmPos0Home = 0;
        //
        cLowerArmPos1LiftBox = 0;
        cUpperArmPos1LiftBox = -120;
        //
        cLowerArmPos2LiftArm = -450;
        cUpperArmPos2LiftArm = -120;
        //
        cLowerArmPos3BackScore = -1500;
        cUpperArmPos3BackScore = -440;
        //
        cLowerArmPos4FrontScore = -700;
        cUpperArmPos4FrontScore = 750;
        //
        cLowerArmPos5Hang = -1200;
        cUpperArmPos5Hang = 850;
        cLowerArmPositions = JavaUtil.createListWith(cLowerArmPos0Home, cLowerArmPos1LiftBox, cLowerArmPos2LiftArm, cLowerArmPos3BackScore, cLowerArmPos4FrontScore, cLowerArmPos5Hang);
        cUpperArmPositions = JavaUtil.createListWith(cUpperArmPos0Home, cUpperArmPos1LiftBox, cUpperArmPos2LiftArm, cUpperArmPos3BackScore, cUpperArmPos4FrontScore, cUpperArmPos5Hang);
        cArmPositionNames = JavaUtil.makeListFromText("Home,Lift Box,Lift Arm,Back Score, Front Score,Hang", ",");
        armMoveToHome = false;
        armMoveToFront = false;
        armMoveToBack = false;
        armMoveToHang = false;
        // Servo speeds for the cassette
        cCasFrontToBack = 0.8;
        cCasBackToFront = 0.2;
        cCasStop = 0.5;
        // Distance to detect pixels in the cassette (cm)
        cPixelPos1Dist = 1;
        cPixelPos2Dist = 10;
        // Intake motor speeds
        cIntakeIn = -1;
        cIntakeOut = 1;
        // Object location enumerations
        cObjectLocationUnknown = 0;
        cObjectLocationLeft = 1;
        cObjectLocationCenter = 2;
        cObjectLocationRight = 3;
        // Max x value for an object on the left spike
        cXvalueForLeftToCenterObject = 200;
        // Maximum time to run image recognition to discover the prop
        cMaxObjectSearchTimeMs = 2000;
        // How long to run the servo and intake when dropping the pixel
        cPixelDropRunTimeMs = 2000;
        cPixelFrontScoreRunTimeMs = 2000;
        // Initialize Local Variables
        autonState = 0;
        objectLocation = cObjectLocationUnknown;
        pixelDropTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        autonAbort = false;
        modelFilename = "Blue_Prop.tflite";
        // Initialization Routines
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        InitArm();
        Drive.Init("MotDrFrLt", "MotDrFrRt", "MotDrBkLt",
                "MotDrBkRt", cCountsPerInch, cDriveBoosted, cDriveNormal, cDriveSlow);
        InitCassette();
        Intake.Init("MotPxlIntk", cIntakeIn, cIntakeOut);
        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection.
        USE_WEBCAM = true;
        initTfod2();
        // Wait for the match to begin.
        waitForStart();
        // Useful code to load pixels before we run. DISABLE FOR COMPETITION
        if (false) {
            while (opModeIsActive()) {
                if (ProcessCassette(cCasFrontToBack, cCasFrontToBack, false) == 3) {
                    break;
                }
                Intake.StartIn();
            }
            Intake.Stop();
        }
        opModeTimer.reset();
        // Find the object so we can drive to it
        objectLocation = GetObjectLocation(cMaxObjectSearchTimeMs);
        // If we did not find it, we have no choice but to assume that it was in the position we can't see
        if (objectLocation == cObjectLocationUnknown) {
            objectLocation = cObjectLocationRight;
        }
        while (opModeIsActive()) {
            if (AutonBlueWing()) {
                break;
            }
            if (opModeTimer.milliseconds() >= 27000) {
                autonAbort = true;
                autonState = 400;
                ArmToHome();
                break;
            }
            telemetry.addData("State", autonState);
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
        while (autonAbort && opModeIsActive()) {
            ArmToHome();
            if (autonState >= 500) {
                break;
            }
            telemetry.addData("State", autonState);
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
    }

    /**
     * Describe this function...
     */
    private void InitArm() {
        // Brake the motors when power is zero. This keeps the arm from falling
        MotLwrArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotUprArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotLwrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotUprArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void ProcessArm() {
        double nextArmPosition;

        if (AllowArmChange()) {
            nextArmPosition = ArmControlSM();
            SetLwrArmPos(((Integer) JavaUtil.inListGet(cLowerArmPositions, JavaUtil.AtMode.FROM_START, ((nextArmPosition + 1) - 1), false)).intValue());
            SetUprArmPos(((Integer) JavaUtil.inListGet(cUpperArmPositions, JavaUtil.AtMode.FROM_START, ((nextArmPosition + 1) - 1), false)).intValue());
            telemetry.addData("ArmPos", ((Double) JavaUtil.inListGet(cArmPositionNames, JavaUtil.AtMode.FROM_START, ((nextArmPosition + 1) - 1), false)).doubleValue());
        }
        telemetry.addData("LrArm", MotLwrArm.getCurrentPosition());
        telemetry.addData("UprArm", MotUprArm.getCurrentPosition());
        telemetry.addData("Manual Arm", 123);
    }

    /**
     * Describe this function...
     */
    private void InitCassette() {
        // Set servo direction. Position 2 is reversed
        SrvPxlPos1.setDirection(Servo.Direction.FORWARD);
        SrvPxlPos2.setDirection(Servo.Direction.REVERSE);
        // Disable the LEDs since we only need distance measurements
        SenColPxlPos1.enableLed(false);
        SenColPxlPos2.enableLed(false);
        // Ensure the servos are stopped
        SetPixelPos1Dir(cCasStop);
        SetPixelPos2Dir(cCasStop);
    }

    /**
     * Describe this function...
     */
    private int ProcessCassette(double inCassetteDirPos1, double inCassetteDirPos2, boolean inScore) {
        int procCasReturn;
        boolean pixelInPos1;
        boolean pixelInPos2;

        // Are there pixels in the cassette?
        pixelInPos1 = DetectPixelPos1();
        pixelInPos2 = DetectPixelPos2();
        // Return a bitfield for which pixels are present
        procCasReturn = 0;
        if (pixelInPos2) {
            // Set bit 1
            procCasReturn += 2;
            if (inCassetteDirPos2 == cCasFrontToBack && !inScore) {
                // Don't run off the end unless we are scoring on the backdrop
                inCassetteDirPos2 = cCasStop;
            } else if (inCassetteDirPos2 == cCasBackToFront && inCassetteDirPos1 != cCasBackToFront) {
                // Don't push into position 1 if it's not moving in the same direction
                inCassetteDirPos2 = cCasStop;
            }
        }
        if (pixelInPos1) {
            // Set bit 0
            procCasReturn += 1;
            if (inCassetteDirPos1 == cCasFrontToBack && pixelInPos2) {
                // Don't run a pixel into another pixel
                inCassetteDirPos1 = cCasStop;
            }
        }
        LED4.enable(pixelInPos2);
        LED3.enable(!pixelInPos2);
        LED2.enable(pixelInPos1);
        LED1.enable(!pixelInPos1);
        SetPixelPos1Dir(inCassetteDirPos1);
        SetPixelPos2Dir(inCassetteDirPos2);
        return procCasReturn;
    }

    /**
     * Describe this function...
     */
    private void SetLwrArmPos(int inLwrArmPos) {
        MotLwrArm.setTargetPosition(inLwrArmPos);
        MotLwrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotLwrArm.setPower(cLowerArmAutoMotorPwr);
    }

    /**
     * Initialize TensorFlow Object Detection.
     */
    private void initTfod2() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName(modelFilename);
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("prop"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void SetUprArmPos(int inUprArmPos) {
        MotUprArm.setTargetPosition(inUprArmPos);
        MotUprArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotUprArm.setPower(cUpperArmAutoMotorPwr);
    }

    /**
     * Describe this function...
     */
    private int ArmControlSM() {
        if (armMoveToHome) {
            if (armPositionState > 2) {
                armPositionState = 2;
            } else if (armPositionState) {
                armPositionState += -1;
            }
        } else if (armMoveToBack) {
            if (armPositionState < 2) {
                armPositionState += 1;
            } else {
                armPositionState = 3;
            }
        } else if (armMoveToFront) {
            if (armPositionState < 2) {
                armPositionState += 1;
            } else {
                armPositionState = 4;
            }
        } else if (armMoveToHang) {
            if (armPositionState < 2) {
                armPositionState += 1;
            } else {
                armPositionState = 5;
            }
        }
        return armPositionState;
    }

    /**
     * Describe this function...
     */
    private int GetObjectLocation(int inSearchTimeMs) {
        int detectedLocation;
        long timer;
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        // Useful way to skip over object detection for testing
        if (false) {
            return cObjectLocationUnknown;
        }
        // Set a timer so we don't do this forever
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        timer = System.currentTimeMillis();
        detectedLocation = cObjectLocationUnknown;
        while (opModeIsActive()) {
            // Get a list of recognitions from TFOD.
            myTfodRecognitions = myTfodProcessor.getRecognitions();
            telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
            // Iterate through list and call a function to display info for each recognized object.
            for (Recognition myTfodRecognition_item : myTfodRecognitions) {
                myTfodRecognition = myTfodRecognition_item;
                // Display info about the recognition.
                telemetry.addLine("");
                // Display label and confidence.
                // Display the label and confidence for the recognition.
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                // Display position.
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                // Display the position of the center of the detection boundary for the recognition
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            }
            // Push telemetry to the Driver Station.
            telemetry.update();
            // See if we found the object. If we found something, determine which spike it is on and leave
            if (JavaUtil.listLength(myTfodRecognitions) > 0) {
                if (x < cXvalueForLeftToCenterObject) {
                    detectedLocation = cObjectLocationLeft;
                } else {
                    detectedLocation = cObjectLocationCenter;
                }
                break;
            }
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            if (System.currentTimeMillis() - timer >= inSearchTimeMs) {
                break;
            }
            // Share the CPU.
            sleep(20);
        }
        return detectedLocation;
    }

    /**
     * Describe this function...
     */
    private void SetPixelPos1Dir(double inDirection) {
        // Keep Servo position in valid range
        inDirection = Math.min(Math.max(inDirection, 0), 1);
        SrvPxlPos1.setPosition(inDirection);
        telemetry.addData("PixelPos1Servo", inDirection);
    }

    /**
     * Describe this function...
     */
    private boolean AllowArmChange() {
        int upperArmError;
        int lowerArmError;

        upperArmError = Math.abs(MotUprArm.getTargetPosition() - MotUprArm.getCurrentPosition());
        lowerArmError = Math.abs(MotLwrArm.getTargetPosition() - MotLwrArm.getCurrentPosition());
        if (lowerArmError <= 15 && upperArmError <= 15) {
            return true;
        }
        if (!(MotLwrArm.isBusy() || MotUprArm.isBusy())) {
            return true;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private void SetPixelPos2Dir(double inDirection) {
        // Keep Servo position in valid range
        inDirection = Math.min(Math.max(inDirection, 0), 1);
        SrvPxlPos2.setPosition(inDirection);
        telemetry.addData("PixelPos2Servo", inDirection);
    }

    /**
     * Describe this function...
     */
    private boolean DetectPixelPos1() {
        double distPixelPos1;
        boolean detPixelPos1;

        distPixelPos1 = SenColPxlPos1_DistanceSensor.getDistance(DistanceUnit.CM);
        detPixelPos1 = distPixelPos1 < cPixelPos1Dist;
        telemetry.addData("PixelPos1Dist", distPixelPos1);
        telemetry.addData("Pixel1Detect", detPixelPos1);
        return detPixelPos1;
    }

    /**
     * Describe this function...
     */
    private void BadState() {
        telemetry.addData("InvalidState", autonState);
    }

    /**
     * Describe this function...
     */
    private boolean DetectPixelPos2() {
        double distPixelPos2;
        boolean detPixelPos2;

        distPixelPos2 = SenColPxlPos2_DistanceSensor.getDistance(DistanceUnit.CM);
        detPixelPos2 = distPixelPos2 < cPixelPos2Dist;
        telemetry.addData("PixelPos2Dist", distPixelPos2);
        telemetry.addData("Pixel2Detect", detPixelPos2);
        return detPixelPos2;
    }

    /**
     * Describe this function...
     */
    private boolean AutonRedWing() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationLeft) {
                    autonState = 10;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 20;
                } else if (objectLocation == cObjectLocationRight) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(16, -14, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -16, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(30, -3, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationLeft) {
                    autonState = 210;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(34, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    if (!Drive.Busy()) {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 214) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 27, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -19, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(87, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 24, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -22, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    if (!Drive.Busy()) {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 232) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 29, 0);
                        autonState = 300;
                    }
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            if (!Drive.Busy()) {
                ScoreFront();
            }
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            if (!Drive.Busy()) {
                ArmToHome();
            }
        } else if (autonState == 500) {
            if (true) {
                return true;
            }
        } else {
            BadState();
            autonState = 500;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private boolean AutonBlueWing() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationRight) {
                    autonState = 10;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 20;
                } else if (objectLocation == cObjectLocationLeft) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(16, 14, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 15, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(30, 3, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationRight) {
                    autonState = 210;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(30, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    if (!Drive.Busy()) {
                        Drive.Start(73, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 214) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -24, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 19, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(87, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -24, 0);
                        autonState = 300;
                    }
                } else {
                    BadState();
                    autonState = 500;
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 22, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    if (!Drive.Busy()) {
                        Drive.Start(76, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 232) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -29, 0);
                        autonState = 300;
                    }
                }
            } else {
                BadState();
                autonState = 500;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            if (!Drive.Busy()) {
                ScoreFront();
            }
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            if (!Drive.Busy()) {
                ArmToHome();
            }
        } else if (autonState == 500) {
            if (true) {
                return true;
            }
        } else {
            BadState();
            autonState = 500;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private boolean AutonRedBackstage() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationRight) {
                    autonState = 10;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 20;
                } else if (objectLocation == cObjectLocationLeft) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(16, 14, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 15, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(28, 3, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationRight) {
                    autonState = 210;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, -20);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(-20, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    autonState += 1;
                } else if (autonState == 214) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -8, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(-17, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(-28, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    autonState += 1;
                } else if (autonState == 232) {
                    // This is the last state for the left spike
                    autonState = 300;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            ScoreBack();
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            ArmToHome();
        } else if (autonState < 600) {
            if (!Drive.Busy()) {
                Drive.Start(0, -36, 0);
                autonState = 600;
            }
        } else if (autonState == 600) {
            if (!Drive.Busy()) {
                return true;
            }
        } else {
            BadState();
            autonState = 600;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private boolean AutonBlueBackstage() {
        // Need to nest a couple of ifs because this app can't handle huge if-else trees
        if (autonState < 100) {
            // These states all handle driving to the object spike location
            if (autonState == 0) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationLeft) {
                    autonState = 10;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 20;
                } else if (objectLocation == cObjectLocationRight) {
                    autonState = 30;
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 20) {
                if (autonState == 10) {
                    // This is the first state for the left spike
                    if (true) {
                        Drive.Start(13, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 11) {
                    // This is the last state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -15, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 30) {
                if (autonState == 20) {
                    // This is the first state for the center spike
                    if (true) {
                        Drive.Start(24, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 21) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 22) {
                    if (!Drive.Busy()) {
                        Drive.Start(-14, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 23) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, -16, 0);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else if (autonState < 40) {
                if (autonState == 30) {
                    // This is the first state for the right spike
                    if (true) {
                        Drive.Start(30, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 31) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState = 100;
                    }
                } else {
                    BadState();
                    autonState = 600;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike. this is the same for all autons
            if (!Drive.Busy()) {
                PixelDrop();
            }
        } else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (autonState == 200) {
                // Jump to the correct state based on the location
                if (objectLocation == cObjectLocationLeft) {
                    autonState = 210;
                } else if (objectLocation == cObjectLocationCenter) {
                    autonState = 220;
                } else {
                    autonState = 230;
                }
            } else if (autonState < 220) {
                if (autonState == 210) {
                    // This is the first state for the left spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 0, 20);
                        autonState += 1;
                    }
                } else if (autonState == 211) {
                    if (!Drive.Busy()) {
                        Drive.Start(-22, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 212) {
                    if (!Drive.Busy()) {
                        Drive.Start(0, -12, 0);
                        autonState += 1;
                    }
                } else if (autonState == 213) {
                    autonState += 1;
                } else if (autonState == 214) {
                    // This is the last state for the right spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 230) {
                if (autonState == 220) {
                    // This is the first state for the center spike
                    if (!Drive.Busy()) {
                        Drive.Start(0, 8, 0);
                        autonState += 1;
                    }
                } else if (autonState == 221) {
                    if (!Drive.Busy()) {
                        Drive.Start(-17, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 222) {
                    // This is the last state for the center spike
                    if (!Drive.Busy()) {
                        autonState = 300;
                    }
                }
            } else if (autonState < 240) {
                if (autonState == 230) {
                    // This is the first state for the right spike
                    if (!Drive.Busy()) {
                        Drive.Start(-31, 0, 0);
                        autonState += 1;
                    }
                } else if (autonState == 231) {
                    autonState += 1;
                } else if (autonState == 232) {
                    // This is the last state for the left spike
                    autonState = 300;
                }
            } else {
                BadState();
                autonState = 600;
            }
        } else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop. this is the same for two autons
            ScoreBack();
        } else if (autonState < 500) {
            // These 400 level states handle returning the arm home. this is the same for all autons
            ArmToHome();
        } else if (autonState < 600) {
            if (!Drive.Busy()) {
                Drive.Start(0, 32, 0);
                autonState = 600;
            }
        } else if (autonState == 600) {
            if (!Drive.Busy()) {
                return true;
            }
        } else {
            BadState();
            autonState = 600;
        }
        return false;
    }

    /**
     * Describe this function...
     */
    private void ScoreBack() {
        if (autonState == 300) {
            armMoveToBack = true;
            ProcessArm();
            autonState += 1;
        } else if (autonState == 301) {
            ProcessArm();
            if (armPositionState == 3 && AllowArmChange()) {
                armMoveToBack = false;
                autonState += 1;
            }
        } else if (autonState == 302) {
            if (!Drive.Busy()) {
                Drive.Start(-6, 0, 0);
                autonState += 1;
            }
        } else if (autonState == 303) {
            if (!Drive.Busy()) {
                foo = ProcessCassette(cCasBackToFront, cCasBackToFront, true);
                pixelDropTimer.reset();
                autonState += 1;
            }
        } else if (autonState == 304) {
            if (pixelDropTimer.milliseconds() >= cPixelFrontScoreRunTimeMs) {
                foo = ProcessCassette(cCasStop, cCasStop, false);
                Drive.Start(4, 0, 0);
                autonState = 400;
            }
        } else {
            BadState();
            autonState = 400;
        }
    }

    /**
     * Describe this function...
     */
    private void ScoreFront() {
        if (autonState == 300) {
            armMoveToFront = true;
            ProcessArm();
            autonState += 1;
        } else if (autonState == 301) {
            ProcessArm();
            if (armPositionState == 4 && AllowArmChange()) {
                armMoveToFront = false;
                autonState += 1;
            }
        } else if (autonState == 302) {
            if (!Drive.Busy()) {
                Drive.Start(8, 0, 0);
                autonState += 1;
            }
        } else if (autonState == 303) {
            if (!Drive.Busy()) {
                foo = ProcessCassette(cCasFrontToBack, cCasFrontToBack, true);
                pixelDropTimer.reset();
                autonState += 1;
            }
        } else if (autonState == 304) {
            if (pixelDropTimer.milliseconds() >= cPixelFrontScoreRunTimeMs) {
                foo = ProcessCassette(cCasStop, cCasStop, false);
                Drive.Start(-6, 0, 0);
                autonState = 400;
            }
        } else {
            BadState();
            autonState = 400;
        }
    }

    /**
     * Describe this function...
     */
    private void PixelDrop() {
        // Drop the pixel on the spike
        if (autonState == 100) {
            // Reverse the intake
            Intake.StartOut();
            // Run one pixel out of the cassette
            foo = ProcessCassette(cCasBackToFront, cCasStop, false);
            // Start a timer since we can't easily detect whether a pixel has exited the intake
            pixelDropTimer.reset();
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            pixelDropTimeStart = System.currentTimeMillis();
            autonState += 1;
        } else if (autonState == 101) {
            // Wait for our hardcoded timer to elapse
            // Get the current time in milliseconds. The value returned represents
            // the number of milliseconds since midnight, January 1, 1970 UTC.
            if (System.currentTimeMillis() - pixelDropTimeStart >= cPixelDropRunTimeMs) {
                // Stop the intake and cassette
                Intake.Stop();
                foo = ProcessCassette(cCasStop, cCasStop, false);
                // Go to the next major step in the auton
                autonState = 200;
            }
        } else {
            // some bad state. stop everything and move on
            BadState();
            autonState = 200;
            Intake.Stop();
            foo = ProcessCassette(cCasStop, cCasStop, false);
        }
    }

    /**
     * Describe this function...
     */
    private void ArmToHome() {
        if (autonState == 400) {
            armMoveToHome = true;
            ProcessArm();
            autonState += 1;
        } else if (autonState == 401) {
            ProcessArm();
            if (armPositionState == 0 && AllowArmChange()) {
                armMoveToHome = false;
                autonState = 500;
            }
        } else {
            BadState();
            autonState = 500;
        }
    }
}
