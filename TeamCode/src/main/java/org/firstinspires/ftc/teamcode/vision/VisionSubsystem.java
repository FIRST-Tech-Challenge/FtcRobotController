package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

/**
 * Owns one webcam and two FTC SDK processors: color blob detection and AprilTags.
 * Construct once per OpMode; call close() when the OpMode finishes.
 */
public final class VisionSubsystem implements AutoCloseable {
    private final ColorBlobLocatorProcessor colorProcessor;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal portal;

    public VisionSubsystem(HardwareMap hardwareMap) {
        colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(VisionConfig.TARGET_COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(5)
                .build();

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, VisionConfig.WEBCAM_NAME))
                .setCameraResolution(new Size(
                        VisionConfig.FRAME_WIDTH_PX, VisionConfig.FRAME_HEIGHT_PX))
                .addProcessor(colorProcessor)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /** Returns the largest sufficiently big color region, or null if none is found. */
    public TargetDetection findLargestColorTarget() {
        ColorBlobLocatorProcessor.Blob best = null;
        int bestArea = VisionConfig.MIN_BLOB_AREA_PX - 1;

        for (ColorBlobLocatorProcessor.Blob blob : colorProcessor.getBlobs()) {
            int area = blob.getContourArea();
            if (area >= VisionConfig.MIN_BLOB_AREA_PX && area > bestArea) {
                best = blob;
                bestArea = area;
            }
        }
        if (best == null) {
            return null;
        }

        RotatedRect boundingBox = best.getBoxFit();
        return new TargetDetection(
                VisionConfig.TARGET_LABEL,
                boundingBox.center.x,
                boundingBox.center.y,
                best.getContourArea(),
                VisionConfig.FRAME_WIDTH_PX);
    }

    /** Returns individual tags; SDK 12 also supports multi-tag cluster detections. */
    public List<AprilTagSingleDetection> getAprilTags() {
        List<AprilTagSingleDetection> tags = new ArrayList<>();
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                tags.add((AprilTagSingleDetection) detection);
            }
        }
        return tags;
    }

    @Override
    public void close() {
        portal.close();
    }
}
