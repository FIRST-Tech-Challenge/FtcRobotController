package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RobotCameraTest", group = "Test")
public class robotCameraTest extends ConeDetection{
    @Override
    public void runOpMode() throws InterruptedException
    {
        detectGrab();
        initialize();
        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("distance", coneDetectionPipeline.distance);
    }
}
