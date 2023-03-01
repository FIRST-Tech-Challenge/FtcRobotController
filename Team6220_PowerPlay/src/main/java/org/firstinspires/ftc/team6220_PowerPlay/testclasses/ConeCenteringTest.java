package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "ConeCenteringTest", group = "Camera testing")
public class ConeCenteringTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize new cameras due to funny init method
        int robotCameraStream = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"),robotCameraStream);
        robotCameraPipeline = new RobotCameraPipeline();
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
        robotCameraPipeline.invertRange(false);
        telemetry.addLine("6220 rap at worlds");
        waitForStart();

        //telemetry
        while(opModeIsActive()){
            telemetry.addData("xMotorPower", pixelsToMotorPower(400-robotCameraPipeline.xPosition));
            telemetry.addData("yMotorPower", widthToMotorPower(robotCameraPipeline.width));
            telemetry.update();
        }
    }
}
