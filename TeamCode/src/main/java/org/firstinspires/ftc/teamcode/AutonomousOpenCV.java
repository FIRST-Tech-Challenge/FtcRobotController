package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Autonomous - Generic", group="Linear Opmode")
@Disabled
public class AutonomousOpenCV extends LinearOpMode {
    private OpenCvWebcam webcam;
    private CircleDetection circleDetection;
    protected DrivingFunctions df;
    protected ServoFunctions sf;
    protected boolean detectionRed = false; // whether to detect a red ball (if false detects blue)
    protected boolean runBallDetectionTest = false;
    protected boolean runEncoderTest = false;
    protected boolean runAutoDrivingTest = false;
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.35;

    private void DetectBallPosition(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetection.CircleFound() && tries < timeoutInSeconds * 10) {
            sleep(100);
            tries++;
            UpdateCircleDetectionTelemetry();
        }
        webcam.stopStreaming();
        if (!circleDetection.CircleFound())
            circleDetection.SetBallPosition(CircleDetection.BallPosition.LEFT); // Ball not found, makes a guess to the left
    }
    private void Initialize() {
        df = new DrivingFunctions(this);
        sf = new ServoFunctions(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetection(detectionRed);
        webcam.setPipeline(circleDetection);
        webcam.setMillisecondsPermissionTimeout(3000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }
    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        if (runAutoDrivingTest) {
            RunAutoDrivingTest();
            return;
        }

        if (runBallDetectionTest) {
            RunBallDetectionTest();
            return;
        }
        if (runEncoderTest) {
            RunEncoderTest();
            return;
        }

        DetectBallPosition(5);

        if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.LEFT)
        {
            PushPixelLeft();
        }
        else if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.CENTER)
        {
            PushPixelCenter();
        }
        else if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.RIGHT)
        {
            PushPixelRight();
        }
    }
    protected void PushPixelRight()
    {
        df.DriveStraight(DRIVE_SPEED, 20, 0, false);
        df.TurnToHeading(TURN_SPEED, -60); // Negative angles turn to the right
        df.DriveStraight(DRIVE_SPEED, 10, -60, false);
        df.DriveStraight(DRIVE_SPEED, -12, -60, false);
    }
    protected void PushPixelLeft()
    {
        df.DriveStraight(DRIVE_SPEED, 20, 0, false);
        df.TurnToHeading(TURN_SPEED, 60); // Positive angles turn to the left
        df.DriveStraight(DRIVE_SPEED, 10, 60, false);
        df.DriveStraight(DRIVE_SPEED, -12, 60,false);
    }
    private void PushPixelCenter()
    {
        df.DriveStraight(DRIVE_SPEED, 35, 0, false);
        df.DriveStraight(DRIVE_SPEED, -15, 0, false);
        df.TurnToHeading(TURN_SPEED, -90);
        df.DriveStraight(DRIVE_SPEED, 40, -90, false);
    }
    protected void finalize()
    {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
    private void UpdateCircleDetectionTelemetry()
    {
        if(!runEncoderTest) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Circles detected: ", "%d", circleDetection.NumCirclesFound());
            telemetry.addData("Circle center = ", "%4.0f, %4.0f", circleDetection.CircleCenter().x, circleDetection.CircleCenter().y);
            telemetry.addData("Ball Position: ", "%s", circleDetection.GetBallPosition());
        }
        telemetry.update();
    }

    // Functions to run tests
    private void RunBallDetectionTest() {
        while (opModeIsActive()) {
            sleep(100);
            UpdateCircleDetectionTelemetry();
        }
    }
    private void RunEncoderTest()
    {
        webcam.stopStreaming();
        df.TestEncoders();
    }
    protected void RunAutoDrivingTest()
    {
        df.DriveStraight(DRIVE_SPEED, 20, 0, false);
        df.DriveStraight(DRIVE_SPEED, -20, 0, false);
        df.TurnToHeading(TURN_SPEED, -60); // Positive angles turn to the left
        df.DriveStraight(DRIVE_SPEED, 10, -60, false);
        df.DriveStraight(DRIVE_SPEED, -12, -60, false);
    }
}