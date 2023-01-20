package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetectionPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@Autonomous(name = "AprilTagTest")
public class AprilTagTest extends BaseAutonomous {

    public AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public OpenCvCamera robotCamera;

    public int cameraMonitorViewId;

    int signal;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        robotCamera.setPipeline(aprilTagDetectionPipeline);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        signal = detectSignal();

        waitForStart();

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);
    }
}
