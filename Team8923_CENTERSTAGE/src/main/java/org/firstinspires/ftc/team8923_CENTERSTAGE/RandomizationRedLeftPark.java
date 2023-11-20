package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationRedLeftPark")

public class RandomizationRedLeftPark extends BaseAutonomous {
    public void runOpMode() {
        // initialize the webcam and openCV pipeline
        myColorDetection.init();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // print detection status and coordinates of largest object
            telemetry.addLine("Detection")
                    .addData(" ", myColorDetection.targetDetected)
                    .addData("x", myColorDetection.targetPoint.x)
                    .addData("y", myColorDetection.targetPoint.y);

            telemetry.addData("Frame Count", myColorDetection.robotCamera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", myColorDetection.robotCamera.getFps()));
            telemetry.addData("Total frame time ms", myColorDetection.robotCamera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", myColorDetection.robotCamera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", myColorDetection.robotCamera.getOverheadTimeMs());
            telemetry.update();

            myColorDetection.robotCamera.stopStreaming();
            myColorDetection.robotCamera.closeCameraDevice();
        }
    }
}