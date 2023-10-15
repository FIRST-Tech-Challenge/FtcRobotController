package org.firstinspires.ftc.teamcode.TANVIII;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AprilTagTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        //builder method init
        AprilTagProcessor myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

         */

        // Create the AprilTag processor the easy way.
        AprilTagProcessor myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);


        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();
            telemetry.addLine(String.valueOf(myAprilTagDetections.size()));
            telemetry.update();

            // Cycle through through the list and process each AprilTag
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    // Now take action based on this tag's ID code, or store info for later action.
                    telemetry.addData("ID", "%d (%s)", detection.id, detection.metadata.name);
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
                telemetry.update();
            }
        }
    }
}