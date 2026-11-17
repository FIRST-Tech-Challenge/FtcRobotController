package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Finds yellow pollen, blue and red objects, and AprilTags at the same time.
 * Configure a webcam in the Robot Configuration with the name "Webcam 1".
 */
@Disabled
@TeleOp(name = "Vision: Pollen, Colors & AprilTags", group = "Vision")
public class PollenAndAprilTagDetection extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int MIN_BLOB_AREA = 80;
    private static final int MAX_BLOB_AREA = 20000;

    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor yellowLocator = createColorLocator(ColorRange.YELLOW);
        ColorBlobLocatorProcessor blueLocator = createColorLocator(ColorRange.BLUE);
        ColorBlobLocatorProcessor redLocator = createColorLocator(ColorRange.RED);
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(320, 240))
                .addProcessors(yellowLocator, blueLocator, redLocator, aprilTagProcessor)
                .build();

        telemetry.addLine("Detecting yellow pollen, blue/red objects, and AprilTags.");
        telemetry.addLine("Camera preview is available from the Driver Station.");
        telemetry.addLine("Press Start to begin telemetry.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            reportColorBlobs("Yellow pollen", yellowLocator.getBlobs());
            reportColorBlobs("Blue objects", blueLocator.getBlobs());
            reportColorBlobs("Red objects", redLocator.getBlobs());

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            telemetry.addData("AprilTags", detections.size());
            for (AprilTagDetection detection : detections) {
                if (!(detection instanceof AprilTagSingleDetection)) continue;
                AprilTagSingleDetection tag=(AprilTagSingleDetection)detection;
                if (tag.metadata != null && tag.ftcPose != null) {
                    telemetry.addData("Tag " + tag.id,
                            "%s | range %.1f in | bearing %.1f deg",
                            tag.metadata.name,
                            tag.ftcPose.range,
                            tag.ftcPose.bearing);
                } else {
                    telemetry.addData("Tag " + tag.id, "visible (pose unavailable)");
                }
            }

            telemetry.update();
            sleep(50);
        }

        visionPortal.close();
    }

    private ColorBlobLocatorProcessor createColorLocator(ColorRange colorRange) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
    }

    private void reportColorBlobs(String label, List<ColorBlobLocatorProcessor.Blob> blobs) {
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA,
                MAX_BLOB_AREA,
                blobs);

        telemetry.addData(label, "%d detected", blobs.size());
        int shown = Math.min(blobs.size(), 3);
        for (int i = 0; i < shown; i++) {
            RotatedRect box = blobs.get(i).getBoxFit();
            telemetry.addData(label + " " + (i + 1), "center (%d, %d), area %d px",
                    (int) box.center.x,
                    (int) box.center.y,
                    blobs.get(i).getContourArea());
        }
    }
}
