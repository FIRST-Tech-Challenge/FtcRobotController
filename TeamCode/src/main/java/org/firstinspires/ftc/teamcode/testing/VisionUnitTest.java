package org.firstinspires.ftc.teamcode.testing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class VisionUnitTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()){
            if(tagProcessor.getDetections().size() > 0){
                for(AprilTagDetection tag:tagProcessor.getDetections()){
                    if (tag.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
                    }
                }

                //telemetry.addData("Pose", tag.robotPose.toString());
                //telemetry.addData("y", tag.ftcPose.y);
                //telemetry.addData("z", tag.ftcPose.z);
                //telemetry.addData("roll", tag.ftcPose.roll);
                //telemetry.addData("pitch", tag.ftcPose.pitch);
                //telemetry.addData("yaw", tag.ftcPose.yaw);

                telemetry.update();
            }
        }
    }
}
