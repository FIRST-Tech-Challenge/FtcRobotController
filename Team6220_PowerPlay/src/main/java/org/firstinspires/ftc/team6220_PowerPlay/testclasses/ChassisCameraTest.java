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

@Autonomous(name = "Chassis camera test", group = "Camera testing")
public class ChassisCameraTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize new cameras due to funny init method
        int robotCameraStream = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"),robotCameraStream);
        robotCameraPipeline = new RobotCameraPipeline();

        //Set camera ranges (Blue atm)
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);

        //Start camera streaming
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //Set pipeline
        robotCamera.setPipeline(robotCameraPipeline);
        telemetry.addLine("6220 rap at worlds");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("width", robotCameraPipeline.width);
            telemetry.addData("positionX", robotCameraPipeline.xPosition);
            telemetry.addData("positionY", robotCameraPipeline.yPosition);
            telemetry.addData("distance from center", (400-robotCameraPipeline.xPosition));
            telemetry.addData("L-H ratio", (robotCameraPipeline.width / robotCameraPipeline.height));
            telemetry.update();
        }
    }
}
