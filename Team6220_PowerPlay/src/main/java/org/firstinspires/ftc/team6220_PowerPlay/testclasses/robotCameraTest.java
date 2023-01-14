package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RobotCameraTest", group = "Test")
public class robotCameraTest extends ConeDetection{
    int[] lowerBlue = {42, 128, 114};
    int[] upperBlue = {168, 242, 255};
    @Override
    //♥sabida♥
    public void runOpMode() throws InterruptedException
    {
        detectGrab(lowerBlue,upperBlue);
        initialize();
        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("distance", coneDetectionPipeline.distance);
    }
}
