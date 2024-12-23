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

import java.util.List;

/**
 * The Camera subsystem uses FTC's VisionPortal to detect color blobs
 * (either Red or Blue) and estimate their orientation (angle).
 */
public class Camera {

    // ---------------------------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------------------------
    private static final String SUBSYSTEM_NAME = "Camera";

    // ---------------------------------------------------------------------------------------------
    // State Variables
    // ---------------------------------------------------------------------------------------------
    private double angle     = 0;   // Current angle
    private double prevAngle = 0;   // Previous angle
    private double position  = 0;   // Current position (some user-defined metric)
    private double prevPosition = 0; // Previous position

    private final boolean isDebugMode;
    private final boolean isRedAlliance;
    private boolean isSpecimen;      // Whether we're looking for a specimen or not (unused in this class, but stored)

    // ---------------------------------------------------------------------------------------------
    // VisionPortal and Processor
    // ---------------------------------------------------------------------------------------------
    private VisionPortal portal;
    private ColorBlobLocatorProcessor blobLocatorProcessor;

    // ---------------------------------------------------------------------------------------------
    // OpMode & Telemetry
    // ---------------------------------------------------------------------------------------------
    private final OpMode opMode;
    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------------------------
    /**
     * Constructs a Camera subsystem to detect either Red or Blue blobs.
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
     * Initializes the vision portal and the blob locator processor.
     */
    private void initVision() {
        // Build a ColorBlobLocatorProcessor with appropriate color range (Red or Blue).
        blobLocatorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(isRedAlliance ? ColorRange.RED : ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(5)
                .build();

        // Build a VisionPortal using the configured processor.
        portal = new VisionPortal.Builder()
                .addProcessor(blobLocatorProcessor)
                .setCameraResolution(new Size(320, 240))    // Adjust resolution as needed
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                "Vision Initialization", "Completed");
    }

    // ---------------------------------------------------------------------------------------------
    // Main Functionality
    // ---------------------------------------------------------------------------------------------
    /**
     * Retrieves the angle of the largest detected color blob, if any.
     *
     * @return the current angle in degrees (or the previous position if no blob is detected).
     */
    public double getAngle() {
        prevAngle = angle;
        prevPosition = position;

        // Retrieve all detected blobs
        List<ColorBlobLocatorProcessor.Blob> blobs = blobLocatorProcessor.getBlobs();

        // Filter out very small blobs, e.g., areas < 50 or > 20,000
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

        if (blobs.isEmpty()) {
            // No blobs detected, return previous position to avoid jump
            DebugUtils.logDebug(telemetry, isDebugMode, SUBSYSTEM_NAME,
                    "No Blobs Detected", "Returning Previous Position: " + prevPosition);
            return prevPosition;
        }

        // Take the first blob (presumably the largest)
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
        // Here we do (optimizedAngle/180) + 0.5, as an example
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

        return angle;
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
                "Specimen Set", specimen);
    }

    public double getCurrentAngle() {
        return angle;
    }

    public double getPreviousAngle() {
        return prevAngle;
    }

    public double getPosition() {
        return position;
    }

    public double getPreviousPosition() {
        return prevPosition;
    }

    public VisionPortal getPortal() {
        return portal;
    }

    public ColorBlobLocatorProcessor getBlobLocatorProcessor() {
        return blobLocatorProcessor;
    }
}
