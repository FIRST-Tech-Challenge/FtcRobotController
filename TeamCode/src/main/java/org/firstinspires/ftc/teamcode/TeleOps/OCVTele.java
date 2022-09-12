package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.TestPipeline;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "OCV")
public class OCVTele extends LinearOpMode {

    OpenCvWebcam OCV;

    @Override
    public void runOpMode() throws InterruptedException {

        TestPipeline TP = new TestPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OCV = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        OCV.setPipeline(TP);

        OCV.setMillisecondsPermissionTimeout(2500);
        OCV.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                OCV.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                 telemetry.addLine("not open");
            }
            });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Frame Count", OCV.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", OCV.getFps()));
            telemetry.addData("Total frame time ms", OCV.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", OCV.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", OCV.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", OCV.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a){
                OCV.stopStreaming();
                sleep(100);

            }

        }


        }




    }



