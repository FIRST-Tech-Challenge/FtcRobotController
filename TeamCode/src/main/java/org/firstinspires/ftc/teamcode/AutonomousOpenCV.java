package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private DrivingFunctions df;
    protected boolean detectionRed = false; // whether to detect a red ball (if false detects blue)

    private ElapsedTime runtime = new ElapsedTime();

    private void Initialize()
    {
        df = new DrivingFunctions(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetection(telemetry, detectionRed);
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

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        int tries = 0;
        // Waits until the camera detects the ball, up to 5 seconds
        while(opModeIsActive() && circleDetection.ballPosition == BallPosition.UNDEFINED && tries < 50) {
            sleep(100);
            tries++;
            UpdateTelemetry();
        }

        if(circleDetection.ballPosition == BallPosition.UNDEFINED)
            circleDetection.ballPosition = BallPosition.LEFT; // Ball not found, makes a guess to the left

        df.driveStraight(0.3, 35, 0.0);
        sleep(1000);

        UpdateTelemetry();
        StopStreaming();
    }

    private void StopStreaming()
    {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private void UpdateTelemetry()
    {
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("Circles detected: ", "%d", circleDetection.numCirclesFound);
        telemetry.addData("Circle center = ", "%4.0f, %4.0f", circleDetection.circleCenter.x, circleDetection.circleCenter.y);
        telemetry.addData("Ball Position: ", "%s", circleDetection.ballPosition);
        telemetry.update();
    }
    enum BallPosition {LEFT, CENTER, RIGHT, UNDEFINED};
}