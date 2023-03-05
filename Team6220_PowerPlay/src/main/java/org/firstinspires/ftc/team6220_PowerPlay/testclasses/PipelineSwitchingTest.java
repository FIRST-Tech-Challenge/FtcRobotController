package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "PipelineSwitchingTest")
public class PipelineSwitchingTest extends BaseAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // creates camera streams
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // creates cameras
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        grabberCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "GrabberCamera"), cameraMonitorViewId2);
        // creates pipelines
        GrabberCameraPipeline grabberCameraPipeline = new GrabberCameraPipeline();
        RobotCameraPipeline robotCameraPipeline = new RobotCameraPipeline();
        grabberCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        // starts streaming camera
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        // sets camera to pipeline
        robotCamera.setPipeline(grabberCameraPipeline);
        waitForStart();
        robotCamera.closeCameraDevice();
        while (opModeIsActive()){
            // creates camera stream again
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            // creates camera again
            robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
            robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    robotCamera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {}
            });
            robotCamera.setPipeline(robotCameraPipeline);
            grabberCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    grabberCamera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {}
            });
            grabberCamera.setPipeline(grabberCameraPipeline);
            telemetry.addData("grabberCamera", grabberCameraPipeline.xPosition);
            telemetry.addData("robotCamera", robotCameraPipeline.xPosition);
            telemetry.update();
        }
    }
}
