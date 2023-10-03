package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Autonomous - Red ball", group="Linear Opmode")
public class AutonomousOpenCV extends LinearOpMode {
    private OpenCvWebcam webcam;
    private CircleDetection circleDetection;
    private DrivingFunctions df;

    private ElapsedTime runtime = new ElapsedTime();

    private void Initialize()
    {
        df = new DrivingFunctions(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetection(telemetry, false);                            
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
        /*while(opModeIsActive()){//circleDetection.ballPosition == BallPosition.UNDEFINED && tries < 50) {
            sleep(100);
            tries++;
            UpdateTelemetry();
        } */

        df.rotateDegrees(0.5,90);
        //df.wait(300);
        df.rotateDegrees(0.5,179);
        //df.wait(300);
        df.rotateDegrees(0.5,-30);
        //df.wait(300);
        df.rotateDegrees(0.5,-179);

        df.rotateFeildCentric(0.5,90);
        //df.wait(300);
        df.rotateFeildCentric(0.5,179);
        //df.wait(300);
        df.rotateFeildCentric(0.5,-90);
        //df.wait(300);
        df.rotateFeildCentric(0.5,-179);
        //df.waitStopped(1000);








        if(circleDetection.ballPosition == BallPosition.UNDEFINED)
            circleDetection.ballPosition = BallPosition.LEFT; // Ball not found, makes a guess to the left

        //df.driveForward(0.3,500);

        if(circleDetection.ballPosition == BallPosition.LEFT) {
            //df.driveForward(0.3,400);
            //df.strafeRight(0.5,250);
        }
        if(circleDetection.ballPosition == BallPosition.RIGHT) {
            //df.driveForward(0.5, 100);
            //df.rotateLeft(1, 500);
            //df.driveForward(0.5,700);
        }
        //df.driveForward(0.5,1000);
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