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

@TeleOp(name = "Grabber camera test", group = "Camera testing")
public class GrabberCameraTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize ranges (for testing)
        Scalar LOWER_BLACK_TEST = new Scalar(50, 0, 0);
        Scalar UPPER_BLACK_TEST = new Scalar(115, 255, 40);
        //init value switching
        int value = 0;
        int range = 0;
        //Initialize new cameras due to funny init method
        int robotCameraStream = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        grabberCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"),robotCameraStream);
        grabberCameraPipeline = new GrabberCameraPipeline();
        grabberCameraPipeline.setRanges(LOWER_BLACK_TEST, UPPER_BLACK_TEST);

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
            if(gamepad1.x)
            {
                value = value == 2 ? 0 : value+1;
                sleep(300);
            }
            if(range == 0)
            {
                if (gamepad1.dpad_up)
                {
                    UPPER_BLACK_TEST.val[value]++;
                    sleep(100);
                } else if (gamepad1.dpad_down)
                {
                    UPPER_BLACK_TEST.val[value]--;
                    sleep(100);
                }
                if(gamepad1.a){
                    range = 1;
                    sleep(300);
                }
            }else{
                if (gamepad1.dpad_up)
                {
                    LOWER_BLACK_TEST.val[value]++;
                    sleep(100);
                } else if (gamepad1.dpad_down)
                {
                    LOWER_BLACK_TEST.val[value]--;
                    sleep(100);
                }
                if(gamepad1.a){
                    range = 0;
                    sleep(300);
                }
            }
            grabberCameraPipeline.setRanges(LOWER_BLACK_TEST, UPPER_BLACK_TEST);
            telemetry.addData("upperHSV", UPPER_BLACK_TEST);
            telemetry.addData("lowerHSV", LOWER_BLACK_TEST);
            telemetry.addData("range (lower = 0, upper = 1)", range);
            telemetry.addData("value 0,1,2 > H,S,V", value);
            telemetry.addData("positionX", grabberCameraPipeline.xPosition);
            telemetry.addData("positionY", grabberCameraPipeline.yPosition);
            telemetry.addData("distance from center", Math.sqrt((Math.pow((grabberCameraPipeline.xPosition-400), 2)+Math.pow((grabberCameraPipeline.yPosition-300), 2))));
            telemetry.update();
        }
    }
}
