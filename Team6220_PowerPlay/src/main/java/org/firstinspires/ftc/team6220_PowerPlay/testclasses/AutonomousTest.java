package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends ConeDetection {

    public RobotCameraPipeline robotCameraPipeline;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);

        robotCameraPipeline = new RobotCameraPipeline();
        robotCameraPipeline.setRanges(Constants.lowerRed, Constants.upperRed);

        camera.setPipeline(robotCameraPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        initialize();
        waitForStart();

        centerRobotCamera(robotCameraPipeline, Constants.CONE_WIDTH, Constants.CONE_CENTERING_KP);

        while (opModeIsActive()) {
            telemetry.addData("xPosition", robotCameraPipeline.xPosition);
            telemetry.addData("yPosition", robotCameraPipeline.yPosition);
            telemetry.addData("width", robotCameraPipeline.detectionWidth);
            telemetry.update();
        }
    }
}
