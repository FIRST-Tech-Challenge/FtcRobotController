package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "GrabberCameraTest", group = "Test")
public class GrabberCameraTest extends GrabberCamera {
    int[] lowerBlack = {100, 150, 20};
    int[] upperBlack = {140, 255, 255};

    @Override
    public void runOpMode() throws InterruptedException {
        detectGrabGrabberCamera(lowerBlack, upperBlack);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("distance to center", GrabberCameraPipeline.distance);
            telemetry.addData("cone bounding box area", GrabberCameraPipeline.coneSize);
            telemetry.addData("cone contour area", GrabberCameraPipeline.coneSizeContourArea);
            telemetry.addData("bounding box and contour area comparison", GrabberCameraPipeline.coneSize - GrabberCameraPipeline.coneSizeContourArea);
            telemetry.addData("bounding box long axis", GrabberCameraPipeline.longAxis);
            telemetry.addData("bounding box short axis", GrabberCameraPipeline.shortAxis);
            telemetry.update();
        }
    }
}
