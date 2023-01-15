package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CVtest", group = "Test")
public class OpenCVTest extends ConeDetection {
    int[] lowerBlue = {100, 150, 20};
    int[] upperBlue = {140, 255, 255};

    @Override
    public void runOpMode() throws InterruptedException {
        detectGrab(lowerBlue, upperBlue);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("distance to center", coneDetectionPipeline.distance);
            telemetry.addData("cone bounding box area", coneDetectionPipeline.coneSize);
            telemetry.addData("cone contour area", coneDetectionPipeline.coneSizeContourArea);
            telemetry.addData("bounding box and contour area comparison", coneDetectionPipeline.coneSize - coneDetectionPipeline.coneSizeContourArea);
            telemetry.addData("bounding box long axis", coneDetectionPipeline.longAxis);
            telemetry.addData("bounding box short axis", coneDetectionPipeline.shortAxis);
            telemetry.update();
        }
    }
}
