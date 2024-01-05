package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

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

    // Motor is 28 ticks per revolution
    // Gear Ratio is 12:1
    // Wheel diameter is 100mm
    double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);

    @Override
    public void init() {

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
        myAprilTagProcessor.setDecimation(1);



        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "gge_backup_cam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640,480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // set camera exposure and gain
        // values used from example code
        //SetAutoCameraExposure();
        // setCameraExposure(2, 250);

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
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, imu, telemetry);
        linearSlideMove = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        state = 0;
        //drive speed limiter
        //double powerFactor = 0.25;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        runtime.reset();
    }

    /** Set the camera gain and exposure. */
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
//            Thread.sleep(50);
//
//        }
//
//        // set exposure and gain
//        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//        sleep(20);
//        GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(gain);
//        sleep(20);
//    }

    /** Sets the camera exposure to automatic */
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

    protected void setFieldPosition(FieldPosition fPos){
        pipeline.setFieldPosition(fPos);
    }

    protected SpikePosition getSpikePosition(){
        return pipeline.getSpikePos();
    }

    protected double getLeftSpikeSaturation(){
        return pipeline.getLeftSpikeSaturation();
    }

    protected double getCenterSpikeSaturation(){
        return pipeline.getCenterSpikeSaturation();
    }

    protected double getRightSpikeSaturation(){
        return pipeline.getRightSpikeSaturation();
    }


    public void GoToAprilTag(int tagNumber) {
        double turnDegrees = 0;
        List<AprilTagDetection> tag = myAprilTagProcessor.getDetections();
        for (int i = 0; i < tag.size(); i++) {


            if (tag.get(i).id == tagNumber) {
                telemetry.addData("tagid found: ", tag.get(i).id);
                telemetry.addData("tagid x: ", tag.get(i).ftcPose.x);
                telemetry.addData("tagid y: ", tag.get(i).ftcPose.y);
                telemetry.addData("tagid yaw: ", tag.get(i).ftcPose.yaw);
                telemetry.update();

                //determines what direction to turn
                if (tagNumber <= 3){
                    turnDegrees = 90;
                } else if (tagNumber > 3) {
                    turnDegrees = -90;
                }


                //moves forward to ensure that the rigging is not in the way
                while (tag.get(i).ftcPose.y>36){
                    if (tag.get(i).ftcPose.y>36) {
                        moveTo.Backwards(20, 25);
                    }

                }

                //Rotate so Gege is square with the back drop
                moveTo.Rotate(turnDegrees);

                //move left or right to line up with backdrop
                while (tag.get(i).ftcPose.x>0.5 && tag.get(i).ftcPose.x<0.5){
                    if (tag.get(i).ftcPose.x>=0) {
                        moveTo.Left(20, 25);
                    } else if (tag.get(i).ftcPose.x<0) {
                        moveTo.Right(20, 25);
                    }
                }

                //moves forward to correct distance to place
                while (tag.get(i).ftcPose.y>12.5 && tag.get(i).ftcPose.y<11.5){
                    if (tag.get(i).ftcPose.y>12) {
                        moveTo.Backwards(20, 25);
                    } else if (tag.get(i).ftcPose.y<12) {
                        moveTo.Forward(20, 25);
                    }
                }

                /**
                 * if (tagNumber <= 3){
                 *     double turnDegrees = -90
                 * }
                 * else-if (tagNumber > 3){
                 *     double turnDegrees = 90
                 * }
                 * */
                /**
                 * while (y>? && y<?)
                 *  if Y is too far
                 *      move ____ 20 ticks
                 *
                 *  else-if y is too close
                 *      move ____ 20 ticks
                 * */
                /**
                 * Rotate so Gege is square with the back drop
                 * Rotate(turnDegrees)
                 * */
                /**
                 * while (x>-0.5 && x<0.5)
                 *  if x is positive
                 *      move right 20 ticks
                 *
                 *  else-if x is negitive
                 *      move left 20 ticks
                 * */
                /**
                 * while (y>? && y<?)
                 *  if Y is too far
                 *      move ____ 20 ticks
                 *
                 *  else-if y is too close
                 *      move ____ 20 ticks
                 * */
            }


        }

    }
}