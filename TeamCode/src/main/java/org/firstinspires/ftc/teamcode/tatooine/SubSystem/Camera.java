package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class Camera {

    // ---------------------------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------------------------
    private static final String SUBSYSTEM_NAME = "Camera";


    // ---------------------------------------------------------------------------------------------
    // State Variables
    // ---------------------------------------------------------------------------------------------
    private double angle = 0;   // Current angle
    private double prevAngle = 0;   // Previous angle
    private double position = 0;   // Current position
    private double prevPosition = 0; // Previous position

    private final boolean isDebugMode;
    private final boolean isRedAlliance;
    private boolean isSpecimen; // Whether to include yellow blobs

    // ---------------------------------------------------------------------------------------------
    // VisionPortal and Processors
    // ---------------------------------------------------------------------------------------------
    private VisionPortal portal;
    private ColorBlobLocatorProcessor allianceProcessor;
    private ColorBlobLocatorProcessor yellowProcessor;

    // ---------------------------------------------------------------------------------------------
    // OpMode & Telemetry
    // ---------------------------------------------------------------------------------------------
    private final OpMode opMode;
    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------------------------
    /**
     * Constructs a Camera subsystem to detect Red, Blue, and Yellow blobs.
     *
     * @param opMode     the active OpMode
     * @param isDebug    whether debug logging is enabled
     * @param isRed      whether we're detecting Red color range (otherwise Blue)
     */
    public Camera(OpMode opMode, boolean isDebug, boolean isRed) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.isDebugMode = isDebug;
        this.isRedAlliance = isRed;
        this.isSpecimen = false;

        initVision();

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Constructor Initialized", "isRed: " + isRedAlliance);
    }

    // ---------------------------------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------------------------------
    /**
     * Initializes the vision portal and the blob locator processors.
     */
    private void initVision() {
        // Alliance Processor
        allianceProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(isRedAlliance ? ColorRange.RED : ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(5)
                .build();

        // Yellow Processor
        yellowProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(5)
                .build();

        // Vision Portal
        portal = new VisionPortal.Builder()
                .addProcessor(allianceProcessor)
                .addProcessor(yellowProcessor)
                .setCameraResolution(new Size(320, 240)) // Adjust resolution as needed
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Vision Initialization", "Completed");
    }

    // ---------------------------------------------------------------------------------------------
    // Main Functionality
    // ---------------------------------------------------------------------------------------------
    /**
     * Retrieves the angle of the largest detected blob, prioritizing alliance blobs,
     * or yellow + alliance blobs when in specimen mode.
     *
     * @return the current angle in degrees (or the previous position if no blob is detected).
     */
    public double getAngle() {
        prevAngle = angle;
        prevPosition = position;

        // Detect blobs based on current mode
        List<ColorBlobLocatorProcessor.Blob> blobs = detectBlobs();

        if (blobs.isEmpty()) {
            // No blobs detected, return previous position to avoid jump
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "No Blobs Detected", "Returning Previous Position: " + prevPosition);
            return prevAngle;
        }

        // Take the largest blob
        ColorBlobLocatorProcessor.Blob blob = blobs.get(0);

        // Extract the rectangle corners
        Point[] pt = new Point[4];
        blob.getBoxFit().points(pt);

        // Compute distances between rectangle corners
        double d03 = Math.sqrt(Math.pow(pt[0].x - pt[3].x, 2) + Math.pow(pt[0].y - pt[3].y, 2));
        double d01 = Math.sqrt(Math.pow(pt[0].x - pt[1].x, 2) + Math.pow(pt[0].y - pt[1].y, 2));

        // Use whichever dimension is shorter to determine angle
        if (d03 < d01) {
            angle = Math.toDegrees(Math.atan2(pt[0].y - pt[3].y, pt[0].x - pt[3].x));
        } else {
            angle = Math.toDegrees(Math.atan2(pt[0].y - pt[1].y, pt[0].x - pt[1].x));
        }

        // Convert the angle to some 'position' metric for your application
        double optimizedAngle = MathUtil.optimizeAngle(angle, prevAngle);
        position = (optimizedAngle / 180) + 0.5;

        // Log debug info
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Detected Angle", angle);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Optimized Angle", optimizedAngle);
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Position", position);

        // Telemetry if needed
        if (isDebugMode) {
            telemetry.addData("pos", position / 5);
            telemetry.addData("ang", optimizedAngle);
        }

        return optimizedAngle;
    }

    /**
     * Detects blobs for alliance and yellow ranges, prioritizing based on `isSpecimen`.
     *
     * @return a list of filtered blobs.
     */
    private List<ColorBlobLocatorProcessor.Blob> detectBlobs() {
        // Dynamically enable or disable the yellow processor based on specimen mode
        if (isSpecimen) {
            if (!portal.getProcessorEnabled(yellowProcessor)) {
                portal.setProcessorEnabled(yellowProcessor, true);
                DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "Yellow Processor", "Enabled");
            }
        } else {
            if (portal.getProcessorEnabled(yellowProcessor)) {
                portal.setProcessorEnabled(yellowProcessor, false);
                DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME, "Yellow Processor", "Disabled");
            }
        }

        // Retrieve blobs from the alliance processor
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(allianceProcessor.getBlobs());

        // If the yellow processor is enabled, add its blobs
        if (portal.getProcessorEnabled(yellowProcessor)) {
            blobs.addAll(yellowProcessor.getBlobs());
        }

        // Filter blobs to remove unwanted small or large blobs
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

        return blobs;
    }

    // ---------------------------------------------------------------------------------------------
    // Getters & Setters
    // ---------------------------------------------------------------------------------------------
    public boolean isDebugMode() {
        return isDebugMode;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    public boolean isSpecimen() {
        return isSpecimen;
    }

    public void setSpecimen(boolean specimen) {
        this.isSpecimen = specimen;
        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Specimen Mode Set", specimen);
    }
}
