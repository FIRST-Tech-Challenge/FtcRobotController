package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RobotCameraTest", group = "Test")
public class robotCameraTest extends ConeDetection{
    int[] lowerBlue = {42, 128, 114};
    int[] upperBlue = {168, 242, 255};
    @Override
    public void runOpMode() throws InterruptedException
    {
        ConeDetectionPipeline coneDetectionPipeline = new ConeDetectionPipeline();
        coneDetectionPipeline.setRanges(lowerBlue,upperBlue);
        detectGrab();
        initialize();
        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("distance", coneDetectionPipeline.distance);
    }
}
