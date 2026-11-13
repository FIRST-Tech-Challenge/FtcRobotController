package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/** Read-only camera test. Does not actuate motors or servos. */
@TeleOp(name = "Vision: Recognition Test", group = "Vision")
public class VisionDebugOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);
        try {
            telemetry.addLine("Webcam initialized. Press START for detections.");
            telemetry.addLine("Camera stream: Driver Station > menu > Camera Stream");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                TargetDetection target = vision.findLargestColorTarget();
                if (target == null) {
                    telemetry.addData("Color target", "none");
                } else {
                    telemetry.addData("Color target", target.getLabel());
                    telemetry.addData("Center (px)", "%.0f, %.0f",
                            target.getCenterXPx(), target.getCenterYPx());
                    telemetry.addData("Area (px^2)", target.getAreaPx());
                    telemetry.addData("Horizontal error", "%.2f",
                            target.getHorizontalError());
                }

                List<AprilTagSingleDetection> tags = vision.getAprilTags();
                telemetry.addData("AprilTags", tags.size());
                for (int index = 0; index < Math.min(3, tags.size()); index++) {
                    AprilTagSingleDetection tag = tags.get(index);
                    String name = tag.metadata != null ? tag.metadata.name : "unknown";
                    telemetry.addData("Tag " + tag.id, name);
                    if (tag.ftcPose != null) {
                        telemetry.addData("Tag " + tag.id + " range (in)",
                                "%.1f", tag.ftcPose.range);
                    }
                }
                telemetry.update();
                sleep(75);
            }
        } finally {
            vision.close();
        }
    }
}
