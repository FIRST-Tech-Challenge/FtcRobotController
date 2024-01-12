package org.firstinspires.ftc.teamcode.opmode.auto;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vision.pipeline.HSVSaturationPipeline;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class AutoBase extends LinearOpMode {

    // <<<<<<end of configurable parameters >>>>>>>>>>>
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 960; // modify for your camera
    protected ElapsedTime runtime = new ElapsedTime(); //
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected IMU imu;
    private RevBlinkinLedDriver blinkinLED;
    protected GamePieceLocation gamepieceLocation;
    Servo leftClaw;
    Servo rightClaw;
    Servo conveyor;
    OpenCvWebcam webcam;

    // Used for managing the AprilTag detection process.
    private AprilTagProcessor myAprilTagProcessor;

    // Used to manage the video source.
    private VisionPortal myVisionPortal;

    HSVSaturationPipeline pipeline;

    Movement moveTo;
    IntakeMovement intake;
    LinearSlideMovement linearSlideMove;
    int state;
    double rightCount = 0;
    double centerCount = 0;
    double leftCount = 0;
    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private DcMotor wrist = null;

    int alignStage = 0;
    double currentX = -2;
    double currentY = 15;

    boolean tagDetected = false;
    boolean aprilTagAligned = false;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;
    double currentAngle = 0;


    // Motor is 28 ticks per revolution
    // Gear Ratio is 12:1
    // Wheel diameter is 100mm
    final static double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "gge_cam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new HSVSaturationPipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                //.setNumThreads(tbd)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // set apriltag resolution decimation factor
        myAprilTagProcessor.setDecimation(2);


        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "gge_backup_cam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // set camera exposure and gain
        // values used from example code
        // SetAutoCameraExposure();
        //setCameraExposure(2, 250);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Adding in PIDF Config values learned from previous testing
        // These may need to be tuned anytime the motor weights or config changes.
        // Set PIDF values thinking of ...
        // ...P as primary force (set second)
        // ...I as smoothing (set last)
        // ...D as deceleration (set third)
        // ...F as holding / static force (set first)
        // For Mecanum drive, 8, 0, 0.5, 5 works well on Tiny
        // ... and 7, 0.2, 0.1, 8 works on Rosie (heavier bot)
        ((DcMotorEx) leftFrontDrive).setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        ((DcMotorEx) leftBackDrive).setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        ((DcMotorEx) rightFrontDrive).setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        ((DcMotorEx) rightBackDrive).setVelocityPIDFCoefficients(10, 0.2, 0.1, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        ((DcMotorEx) leftLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 8);
        ((DcMotorEx) rightLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 8);
        // For Wrist, PIDF values set to reduce jitter
        ((DcMotorEx) wrist).setVelocityPIDFCoefficients(15, 0.2, 0.05, 16);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Somehow this is reversed from the TeleOp Gge program.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(DcMotor.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, conveyor, telemetry);
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, imu, telemetry);
        linearSlideMove = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        state = 0;
        //drive speed limiter
        //double powerFactor = 0.25;

        // Wait for the game to start (driver presses PLAY)

        // Temporarily disabled to allow seeing the april tag detections telemetry
        // telemetry.addData("Status", "Initialized");
        // telemetry.update();

        runtime.reset();
    }

//    /** Set the camera gain and exposure. */
//    public void setCameraExposure(int exposureMS, int gain) {
//
//        // wait until camera in streaming mode
//        while (myVisionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING)
//        {}
//
//        // set exposure control to manual
//        ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
//        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//            //Thread.sleep(50);
//        }
//
//        // set exposure and gain
//        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//        //sleep(20);
//        GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(gain);
//        //sleep(20);
//    }

    /**
     * Sets the camera exposure to automatic
     */
    public void SetAutoCameraExposure() {
        ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Auto);
    }

    public void StopMotors() {
        // Set Powers to 0 for safety and not knowing what they are set to.
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    protected void displayTelemetry(double DirectionNow) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
        telemetry.addData("Target Position", leftFrontDrive.getTargetPosition());
        telemetry.addData("Left Front Pos", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Right Front Pos", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Pos", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Back Pos", rightBackDrive.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.addData("location", gamepieceLocation);
        telemetry.update();
    }

    protected void setFieldPosition(FieldPosition fPos) {
        pipeline.setFieldPosition(fPos);
    }

    protected SpikePosition getSpikePosition() {
        return pipeline.getSpikePos();
    }

    protected double getLeftSpikeSaturation() {
        return pipeline.getLeftSpikeSaturation();
    }

    protected double getCenterSpikeSaturation() {
        return pipeline.getCenterSpikeSaturation();
    }

    protected double getRightSpikeSaturation() {
        return pipeline.getRightSpikeSaturation();
    }


    public boolean GoToAprilTag(int tagNumber) {
        double targetX = 0;
        // The AprilTag is not centered on the LEFT and RIGHT backdrop zones, adjust X targets
        if (tagNumber == 1 || tagNumber == 4) {
            targetX = 0.5;
        } else if (tagNumber == 3 || tagNumber == 6) {
            targetX = -0.5;
        }
        double targetY = 10;
        double targetAngle = 0;

        // Translate the tagNumber requested to know the angle of the backdrop in robot IMU
        if (tagNumber <= 3) {
            targetAngle = -90;
        } else if (tagNumber > 3) {
            targetAngle = 90;
        }

        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Scan for April Tag detections and update current values if you find one.
        List<AprilTagDetection> tag = myAprilTagProcessor.getDetections();
        if (tag != null) {
            for (int i = 0; i < tag.size(); i++) {
                if (tag.get(i) != null) {
                    if (tag.get(i).id == tagNumber) {
                        currentX = tag.get(i).ftcPose.x;
                        currentY = tag.get(i).ftcPose.y;
                        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        tagDetected = true;
                    }
                }
            }
        }

        // Update Telemetry with key data
        telemetry.addData("tags found: ", tag.size());
        telemetry.addData("AlignStage: ", alignStage);
        telemetry.addData("Current X: ", currentX);
        telemetry.addData("Target X: ", targetX);
        telemetry.addData("Current Y: ", currentY);
        telemetry.addData("Target Y: ", targetY);
        telemetry.addData("Current Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Target Angle: ", targetAngle);
        telemetry.update();

        // Like the driver control TeleOp, consider the needed axial, lateral and yaw for
        // the motion needed to get to the April Tag.
        // axial from drive is gamepad1.left_stick_y;
        // lateral from drive is -gamepad1.left_stick_x;
        // yaw from drive is -gamepad1.right_stick_x;

        // Stage 0 - Ensure that motor powers are zeroed and switch to RUN_USING_ENCODER mode.
        if (alignStage == 0) {
            // Motors will bee to be in RUN_USING_ENCODER for this vs. RUN_TO_POSITION mode
            // Refactor this to the Movement class to make a method to switch motors to run
            // on a defined power level.
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Set default to BRAKE mode for control
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Set Powers to 0 for safety and not knowing what they are set to.
            StopMotors();
            // Init variables for motion
            axial = 0;
            lateral = 0;
            yaw = 0;
            // increment alignStage
            alignStage = 1;
        }

        // If no tag is detected, creep backwards.
        if (!tagDetected){
            axial = -0.10;
            aprilTagAligned = false;
        }

        // Square up the robot to the backdrop (from targetAngle above)
        // If the yaw is +, apply -yaw, if the yaw if -, apply +yaw (-right_stick_x in robot mode)
        if (abs (targetAngle - currentAngle) > 2) {
            yaw = -Movement.CalcTurnError(targetAngle, currentAngle) / 45;
            if (yaw > 0.3){
                yaw = 0.3;
            } else if (yaw < -0.3){
                yaw = -0.3;
            }
        } else {
            yaw = 0;
        }

        // Slide laterally to correct for X or right motion
        // If the x distance is > 1 inch off of targetX move left or right accordingly
        // To make the robot go right, reduce the lateral (-left_stick_x in robot mode)
        // To make the robot go left, increase the lateral (-left_stick_x in robot mode)
        if (targetX - currentX > 1) {
            lateral = 0.2;
        } else if (targetX - currentX < -1) {
            lateral = -0.2;
        }else {
            lateral = 0;
        }

        // Back the robot up to the right distance to raise the lift
        if (currentY > targetY) {
            axial = -0.15;
        } else {
            axial = 0;
        }

        // Combine the axial, lateral and yaw factors to be powers
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Apply calculated values to drive motors
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Test to see if we are at all three parts of our desired position and we are aligned.
        if (abs (targetX - currentX) < 1 && currentY < targetY && abs (targetAngle - currentAngle) < 2){
            aprilTagAligned = true;
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }
        return aprilTagAligned;
    }
}

