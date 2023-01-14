package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
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
            telemetry.update();
        }
    }
}
