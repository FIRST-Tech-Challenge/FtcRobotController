package org.firstinspires.ftc.teamcode.opmode.auto;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.util.Size;
import android.view.View;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.VisionProcessorMode;
import org.firstinspires.ftc.teamcode.vision.pipeline.HSVSaturationPipeline;
import org.firstinspires.ftc.teamcode.utility.GamePieceLocation;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutoBase extends OpMode {

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

    HSVSaturationPipeline HSVProcessor;

    WebcamName frontCam;
    WebcamName rearCam;

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
    double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);

    @Override
    public void init() {

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName = null;
//        webcamName = hardwareMap.get(WebcamName.class, "gge_cam"); // put your camera's name here
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        HSVProcessor = new HSVSaturationPipeline();
//
//        webcam.setPipeline(HSVProcessor);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Failed", "");
//                telemetry.update();
//            }
//        });

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

        frontCam = hardwareMap.get(WebcamName.class, "gge_cam");
        rearCam = hardwareMap.get(WebcamName.class, "gge_backup_cam");
        CameraName switchableCamera = ClassFactory.getInstance()
                                        .getCameraManager()
                                        .nameForSwitchableCamera(frontCam,rearCam);

        // create our HSV processor
        HSVProcessor = new HSVSaturationPipeline();

        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(myAprilTagProcessor,HSVProcessor)
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


        // To start, we will be using the front camera
        myVisionPortal.setActiveCamera(frontCam);

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
        ((DcMotorEx) leftFrontDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) leftBackDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) rightFrontDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) rightBackDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        ((DcMotorEx) leftLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);
        ((DcMotorEx) rightLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);
        // For Wrist, PIDF values set to reduce jitter
        ((DcMotorEx) wrist).setVelocityPIDFCoefficients(8, 0, 0, 1);

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
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive,
                            imu, hardwareMap, telemetry,myVisionPortal,frontCam,rearCam,HSVProcessor,myAprilTagProcessor);

        // set the proper camera and processor
        moveTo.selectVisionProcessor(VisionProcessorMode.FRONT_CAMERA_HSV);

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
        HSVProcessor.setFieldPosition(fPos);
    }

    protected SpikePosition getSpikePosition() {
        return HSVProcessor.getSpikePos();
    }

    protected double getLeftSpikeSaturation() {
        return HSVProcessor.getLeftSpikeSaturation();
    }

    protected double getCenterSpikeSaturation() {
        return HSVProcessor.getCenterSpikeSaturation();
    }

    protected double getRightSpikeSaturation() {
        return HSVProcessor.getRightSpikeSaturation();
    }



}

