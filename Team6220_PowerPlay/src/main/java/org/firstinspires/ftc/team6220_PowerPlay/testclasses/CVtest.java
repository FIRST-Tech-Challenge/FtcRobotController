package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.ConeDetection;
import org.firstinspires.ftc.team6220_PowerPlay.ConeDetectionPipeline;

@Autonomous(name = "CVtest", group = "Test")
public class CVtest extends ConeDetection
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        detectGrab();
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("distance to center", ConeDetectionPipeline.distance);
            telemetry.addData("grab boolean", ConeDetectionPipeline.grab);
            telemetry.update();
        }
    }
}
