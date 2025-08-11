package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SampleDetectionPipeline;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.teamcode.SampleDetectionPipeline;

@TeleOp(name = "HopefullyThisWorksOpMode")
public class MultiColorDetectionOpMode extends LinearOpMode {

    OpenCvCamera camera;
    SampleDetectionPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SampleDetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Yellow", "X=%.2f, Y=%.2f, Dist=%.2f in", pipeline.yellowX, pipeline.yellowY, pipeline.yellowDist);
            telemetry.addData("Red",    "X=%.2f, Y=%.2f, Dist=%.2f in", pipeline.redX,    pipeline.redY,    pipeline.redDist);
            telemetry.addData("Blue",   "X=%.2f, Y=%.2f, Dist=%.2f in", pipeline.blueX,   pipeline.blueY,   pipeline.blueDist);
            telemetry.update();
        }
    }
}
