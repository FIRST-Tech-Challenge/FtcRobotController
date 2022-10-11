package org.firstinspires.ftc.teamcode.TeleOps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.TestPipeline;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "EOCV")
public class EOCVTele extends LinearOpMode {

    OpenCvWebcam EOCV;

    @Override
    public void runOpMode() throws InterruptedException {

        TestPipeline TP = new TestPipeline(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        EOCV = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        EOCV.setPipeline(TP);

        EOCV.setMillisecondsPermissionTimeout(2500);
        EOCV.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                EOCV.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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

            telemetry.addData("Frame Count", EOCV.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", EOCV.getFps()));
            telemetry.addData("Total frame time ms", EOCV.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", EOCV.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", EOCV.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", EOCV.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a){
                EOCV.stopStreaming();
                sleep(100);

            }

        }


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(EOCV, 10);

    }



}



