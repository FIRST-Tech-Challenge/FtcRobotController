package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RobotCameraTestJunctionDet", group = "Test")
public class junctionTopDetect extends ConeDetection{
    int[] lowerYellow = {112, 100, 100};
    int[] upperYellow = {124, 255, 255};

    @Override
    public void runOpMode() throws InterruptedException {
        detectGrab(lowerYellow, upperYellow);
        initialize();
        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addData("distance", robotCameraPipeline.distance);
    }
}
