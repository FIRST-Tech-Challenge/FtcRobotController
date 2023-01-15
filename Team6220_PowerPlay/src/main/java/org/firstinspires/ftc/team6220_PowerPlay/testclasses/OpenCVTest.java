package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CVtest", group = "Test")
public class OpenCVTest extends ConeDetection {
    int[] lowerRed = {100, 150, 20};
    int[] upperRed = {140, 255, 255};

    @Override
    public void runOpMode() throws InterruptedException {
        detectGrab(lowerRed, upperRed);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("xpos", coneDetectionPipeline.Xpos);
            telemetry.addData("ypos", coneDetectionPipeline.Ypos);
            telemetry.addData("cone bounding box width", coneDetectionPipeline.coneSize);
            telemetry.update();
        }
    }
}
