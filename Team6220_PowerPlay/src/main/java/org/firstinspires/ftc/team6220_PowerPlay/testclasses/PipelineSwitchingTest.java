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
        // creates cameras
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        // creates pipelines
        GrabberCameraPipeline grabberCameraPipeline = new GrabberCameraPipeline();
        RobotCameraPipeline robotCameraPipeline = new RobotCameraPipeline();
        grabberCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        // starts streaming camera
        startCameraWithPipeline(grabberCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        waitForStart();
        robotCamera.closeCameraDevice();
        while (opModeIsActive()) {
            // creates camera stream again
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            // creates camera again
            robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
            startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        }
    }
}
