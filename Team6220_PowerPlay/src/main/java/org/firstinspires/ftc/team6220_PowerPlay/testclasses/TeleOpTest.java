package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "TeleOpCameraTest")
public class TeleOpTest extends BaseTeleOp {

    @Override
    public void runOpMode() {
        String color = "blue";
        // initialize new cameras due to funny init method
        int robotCameraStream = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"),robotCameraStream);
        robotCameraPipeline = new RobotCameraPipeline();

        // set camera ranges (Blue atm)
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);

        // start camera streaming
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        // set the robot camera to the pipeline
        robotCamera.setPipeline(robotCameraPipeline);

        // wait for start and funnee line
        telemetry.addLine("6220 rap at worlds");
        waitForStart();

        // color range switching based on gamepad inputs
        while(opModeIsActive()){
            if(gamepad1.b){
                robotCameraPipeline.setRanges(Constants.LOWER_RED, Constants.UPPER_RED);
                color = "red";
                robotCameraPipeline.invertRange(true);
            }else if (gamepad1.x){
                robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
                robotCameraPipeline.invertRange(false);
                color = "blue";
            }else if(gamepad1.a){
                robotCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
                color = "black";
                robotCameraPipeline.invertRange(false);
            }else if(gamepad1.y){
                robotCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);
                color = "yellow";
                robotCameraPipeline.invertRange(false);
            }

            //telemetry
            telemetry.addLine(color);
            telemetry.addData("width", robotCameraPipeline.width);
            telemetry.addData("positionX", robotCameraPipeline.xPosition);
            telemetry.addData("positionY", robotCameraPipeline.yPosition);
            telemetry.addData("distance from center", (robotCameraPipeline.xPosition-400));
            telemetry.addData("L-H ratio", (robotCameraPipeline.width / robotCameraPipeline.height));
            telemetry.update();
        }
    }
}
