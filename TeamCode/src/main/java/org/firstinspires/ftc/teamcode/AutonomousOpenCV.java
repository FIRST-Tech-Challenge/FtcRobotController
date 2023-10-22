package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Autonomous - Generic", group="Linear Opmode")
@Disabled
public class AutonomousOpenCV extends LinearOpMode {
    public String ballPosition;
    private OpenCvWebcam webcam;
    protected CircleDetection circleDetection;
    protected DrivingFunctions df;
    protected ServoFunctions sf;
    protected boolean isRed = false; // whether to detect a red ball (if false detects blue)
    protected boolean isNear = false; // whether we start from the near side of the backboard
    protected boolean runBallDetectionTest = false;
    protected boolean runEncoderTest = false;
    protected boolean runAutoDrivingTest = false;
    static final double DRIVE_SPEED = 0.35;
    static final double TURN_SPEED = 0.5    ;

    private void RunBallDetectionTest() {
        while (opModeIsActive()) {
            sleep(100);
            UpdateCircleDetectionTelemetry(0);
        }
    }

    private void DetectBallPosition(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetection.CircleFound() && tries < timeoutInSeconds * 10) {
            sleep(100);
            tries++;
            UpdateCircleDetectionTelemetry(tries);
        }
        if (!circleDetection.CircleFound())
            circleDetection.SetBallPosition(CircleDetection.BallPosition.LEFT); // Ball not found, makes a guess to the left
    }

    private void Initialize() {
        df = new DrivingFunctions(this);
        sf = new ServoFunctions(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetection(isRed);
        webcam.setPipeline(circleDetection);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
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

        double aimingDistance = 0; // if the ball is left, then this is 0, center is 6, right is 12
        double strafeCorrection = 0; // adds removes 2 extra inches depending on what side the ball was

        if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.LEFT)
        {
            ballPosition = "left";
            PushPixelSide(false);
            strafeCorrection = isNear ? -1.5 : -1.5;
            aimingDistance = isRed ? 12 : 0;
        }
        else if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.CENTER)
        {
            ballPosition = "center";
            PushPixelCenter();
            aimingDistance = 6;
            strafeCorrection = isNear ? 0 : -3;
        }
        else if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.RIGHT)
        {
            ballPosition = "right";
            PushPixelSide(true);
            strafeCorrection = isNear ? -1.5 : -1.5;
            aimingDistance = isRed ? 0 : 12;
        }
        if(!isNear)
            CrossField();
        DeliverPixel(aimingDistance, strafeCorrection);
        ParkRobot();
    }

    protected void PushPixelSide(boolean isRight)
    {
        double angle = 0;
        boolean movingAwayFromTruss = (isRight && isNear && isRed) || (isRight && !isNear && !isRed) ||
                (!isRight && !isNear && isRed) || (!isRight && isNear && !isRed);
        if(movingAwayFromTruss)
            angle = isRight ? -23 : 23;
        else
            angle = isRight ? -35 : 35;

        df.DriveStraight(DRIVE_SPEED, 6, 0, false);

        if (!movingAwayFromTruss)
            df.DriveStraight(DRIVE_SPEED, isRight ? -10.5 : 10.5, 0, true);
        df.TurnToHeading(TURN_SPEED,angle);
        df.DriveStraight(DRIVE_SPEED, movingAwayFromTruss ? 25: 32, angle, false);
        df.DriveStraight(DRIVE_SPEED, movingAwayFromTruss ? -13: -18, angle, false);
        df.TurnToHeading(TURN_SPEED,0);
    }
    private void PushPixelCenter()
    {
        df.DriveStraight(DRIVE_SPEED, 34, 0, false);
        df.DriveStraight(DRIVE_SPEED, -17, 0, false);
    }
    private void CrossField()
    {
        df.DriveStraight(DRIVE_SPEED, -13, 0, false);
        df.DriveStraight(DRIVE_SPEED * 1.5, isRed ? 60 : -60, 0, true);
        df.DriveStraight(DRIVE_SPEED, 13, 0, false);
    }
    protected void DeliverPixel(double aimingDistance, double strafeCorrection)
    {
        // Strafe towards the backboard
        df.DriveStraight(DRIVE_SPEED, isRed ? 32 + strafeCorrection : -32 + strafeCorrection, 0, true);
        // In the blue case we need to turn around 180 degrees to deliver the pixel (delivery is on the right of the robot)
        if (!isRed)
            df.TurnToHeading(TURN_SPEED, 180);

        int deliveryHeading = isRed ? 0 : 180;
        // Moves forward or backwards to align with the destination on the board
        df.DriveStraight(DRIVE_SPEED, isRed ? 12 + aimingDistance : 3 - aimingDistance, deliveryHeading, false);
        // Strafes right towards the backboard (almost touching it)
        df.DriveStraight(DRIVE_SPEED * 0.6, 11, deliveryHeading, true);
        sf.PutPixelInBackBoard();
        // Gets away from the board after delivering pixel
        df.DriveStraight(DRIVE_SPEED, -6, deliveryHeading, true);
        // Moves the aiming distance towards the wall, so the robot ends up in the same place regardless of where it delivered the pixel
        df.DriveStraight(DRIVE_SPEED, isRed ? -aimingDistance : aimingDistance , deliveryHeading, false);
    }

    protected void ParkRobot()
    {
        int deliveryHeading = isRed ? 0 : 180;
        df.DriveStraight(DRIVE_SPEED, isRed ? -26 : 14, deliveryHeading, false);
        df.DriveStraight(DRIVE_SPEED, 18, deliveryHeading, true);
    }
    private void RunEncoderTest()
    {
        StopStreaming();
        df.TestEncoders();
    }
     protected void finalize()
    {
        StopStreaming();
    }
    private void StopStreaming()
    {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
    private void UpdateCircleDetectionTelemetry(int tries)
    {
        if(!runEncoderTest) {
            telemetry.addData("Tries: ", tries);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("Frames processed: ", circleDetection.FramesProcessed());
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
    protected void RunAutoDrivingTest()
    {
        //df.DriveStraight(DRIVE_SPEED, 24, 0, false);
        //df.DriveStraight(DRIVE_SPEED, 24, 0, true);
        //df.DriveStraight(DRIVE_SPEED, 10, 90, false);
        //df.DriveStraight(DRIVE_SPEED, 10, -90, true);

        //df.DriveStraight(DRIVE_SPEED, -20, 0, false);

        df.TurnToHeading(TURN_SPEED, -90); // Positive angles turn to the left
        df.TurnToHeading(0.7, 90); // Positive angles turn to the left
        df.TurnToHeading(0.8, 180); // Positive angles turn to the left
        df.TurnToHeading(1.0, 0); // Positive angles turn to the left

        //df.DriveStraight(DRIVE_SPEED, 10, -60, false);
        //df.DriveStraight(DRIVE_SPEED, -12, -60, false);
    }
}