package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class multiCamera extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int [] portalList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        AprilTagProcessor tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalList[0])
                .build();

        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .addProcessor(tagProcessor2)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalList[1])
                .build();


        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor1.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor1.getDetections().get(0);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);

            }
            telemetry.update();

        }
    }
}
