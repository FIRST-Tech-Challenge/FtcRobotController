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

@TeleOp(name = "JunctionCenteringTest", group = "Camera testing")
public class JunctionCenteringTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize new cameras due to funny init method
        int robotCameraStream = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        grabberCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"),robotCameraStream);
        grabberCameraPipeline = new GrabberCameraPipeline();
        grabberCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);

        //Start camera streaming
        grabberCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                grabberCamera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //Set pipeline
        grabberCamera.setPipeline(grabberCameraPipeline);
        telemetry.addLine("6220 rap at worlds");
        waitForStart();

        //telemetry
        while(opModeIsActive()){
            telemetry.addData("xMotorPower", pixelsToMotorPower(grabberCameraPipeline.xPosition-400));
            telemetry.addData("yMotorPower", pixelsToMotorPower(300-grabberCameraPipeline.yPosition));
            telemetry.update();
        }
    }
}
