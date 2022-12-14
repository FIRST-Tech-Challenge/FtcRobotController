package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.ConeDetection;

@Disabled
@Autonomous(name = "CVtest", group = "Test")
public class OpenCVTest extends ConeDetection {

    @Override
    public void runOpMode() throws InterruptedException {
        detectGrab();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("distance to center", coneDetectionPipeline.distance);
            telemetry.addData("grab boolean", coneDetectionPipeline.grab);
            telemetry.update();
        }
    }
}
