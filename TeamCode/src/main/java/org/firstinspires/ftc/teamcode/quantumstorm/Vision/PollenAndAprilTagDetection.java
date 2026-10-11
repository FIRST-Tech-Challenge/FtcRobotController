package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

import java.util.Arrays;
import java.util.List;

/**
 * Searches for blue objects, approaches blue or yellow pollen, backs away and
 * turns away when red is seen, and reports AprilTags at the same time.
 * The approach/search behavior is intentionally slow; tune it on blocks before
 * running near field walls or other robots.
 */
@Autonomous(name = "Auto: Find Blue/Yellow, Avoid Red", group = "Vision")
public class PollenAndAprilTagAuto extends LinearOpMode {

    private static final String WEBCAM_NAME = "limelight 3a";
    // Standard FTC servo commands use 0.0..1.0. These thresholds assume
    // the servo's full commanded travel represents 0..180 degrees.
    private static final double APRILTAG_MODE_MIN_SERVO_DEGREES = 70.0;
    private static final double APRILTAG_MODE_MAX_SERVO_DEGREES = 120.0;
    private static final int FRAME_WIDTH = 320;
    private static final int FRAME_HEIGHT = 240;
    private static final int MIN_BLOB_AREA = 80;
    private static final int MAX_BLOB_AREA = FRAME_WIDTH * FRAME_HEIGHT;

    // Tune these after checking the camera preview and measured object areas.
    private static final double MAX_FORWARD_POWER = 0.22;
    private static final double MAX_STRAFE_POWER = 0.18;
    private static final double SEARCH_TURN_POWER = 0.14;
    private static final double RED_BACKUP_POWER = 0.16;
    private static final double RED_TURN_POWER = 0.16;
    private static final int TARGET_STOP_AREA = 3500;
    private static final int CENTER_TOLERANCE_PIXELS = 18;
    private static final double TAG_SEARCH_TURN_POWER = 0.10;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Servo cameraServo = hardwareMap.get(Servo.class, Constants.LIMELIGHT_TILT_SERVO);

        ColorBlobLocatorProcessor yellowLocator = createColorLocator(ColorRange.YELLOW);
        ColorBlobLocatorProcessor blueLocator = createColorLocator(ColorRange.BLUE);
        ColorBlobLocatorProcessor redLocator = createColorLocator(ColorRange.RED);
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT))
                .addProcessors(Arrays.asList(yellowLocator, blueLocator, redLocator, aprilTagProcessor))
                .build();

        telemetry.addLine("Ready: blue search, yellow/blue approach, red avoidance, AprilTags.");
        telemetry.addLine("Use a clear test area; press Start to move.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                double servoPosition = cameraServo.getPosition();
                double servoDegrees = servoPosition * 180.0;
                boolean aprilTagMode = servoDegrees >= APRILTAG_MODE_MIN_SERVO_DEGREES
                        && servoDegrees <= APRILTAG_MODE_MAX_SERVO_DEGREES;

                List<ColorBlobLocatorProcessor.Blob> yellow = filteredBlobs(yellowLocator);
                List<ColorBlobLocatorProcessor.Blob> blue = filteredBlobs(blueLocator);
                List<ColorBlobLocatorProcessor.Blob> red = filteredBlobs(redLocator);

                if (aprilTagMode) {
                    // In the tag servo range, ignore color blobs and search for tags.
                    List<AprilTagDetection> tags = aprilTagProcessor.getDetections();
                    if (tags.isEmpty()) {
                        drive.drive(0, 0, TAG_SEARCH_TURN_POWER);
                        telemetry.addData("Action", "Servo in tag range: searching for AprilTags");
                    } else {
                        drive.stop();
                        telemetry.addData("Action", "AprilTag found; stopped");
                    }
                // Red avoidance has priority while the servo is in ball-search mode.
                } else if (!red.isEmpty()) {
                    ColorBlobLocatorProcessor.Blob hazard = largest(red);
                    double redX = hazard.getBoxFit().center.x;
                    // Turn away from the side where the red object appears.
                    double turnAway = redX >= FRAME_WIDTH / 2.0
                            ? -RED_TURN_POWER : RED_TURN_POWER;
                    drive.drive(-RED_BACKUP_POWER, 0, turnAway);
                    telemetry.addData("Action", "RED SEEN: backing up and turning away");
                } else {
                    // Blue is the search target. If it is not visible, approach yellow pollen.
                    ColorBlobLocatorProcessor.Blob target = !blue.isEmpty()
                            ? largest(blue)
                            : (!yellow.isEmpty() ? largest(yellow) : null);

                    if (target == null) {
                        // Rotate in place to scan until a blue or yellow blob enters view.
                        drive.drive(0, 0, SEARCH_TURN_POWER);
                        telemetry.addData("Action", "Searching for blue / yellow");
                    } else {
                        RotatedRect box = target.getBoxFit();
                        double horizontalError = box.center.x - FRAME_WIDTH / 2.0;
                        int area = largestArea(target);

                        if (area >= TARGET_STOP_AREA) {
                            drive.stop();
                            telemetry.addData("Action", "Target close; stopped");
                        } else {
                            double strafe = Math.abs(horizontalError) <= CENTER_TOLERANCE_PIXELS
                                    ? 0
                                    : clamp(horizontalError / (FRAME_WIDTH / 2.0) * MAX_STRAFE_POWER,
                                            -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
                            drive.drive(MAX_FORWARD_POWER, strafe, 0);
                            telemetry.addData("Action", "Moving toward %s",
                                    !blue.isEmpty() ? "blue" : "yellow pollen");
                        }
                        telemetry.addData("Target center", "x=%d y=%d area=%d px",
                                (int) box.center.x, (int) box.center.y, area);
                    }
                }

                telemetry.addData("Yellow pollen blobs", yellow.size());
                telemetry.addData("Blue blobs", blue.size());
                telemetry.addData("Red blobs", red.size());
                List<AprilTagDetection> tags = aprilTagProcessor.getDetections();
                telemetry.addData("Camera servo", "%.2f (about %.0f degrees)", servoPosition, servoDegrees);
                telemetry.addData("Vision mode", aprilTagMode ? "AprilTags" : "balls/colors");
                telemetry.addData("AprilTags", tags.size());
                for (AprilTagDetection tag : tags) {
                    if (tag.ftcPose != null) {
                        telemetry.addData("Tag " + tag.id, "range %.1f in, bearing %.1f deg",
                                tag.ftcPose.range, tag.ftcPose.bearing);
                    } else {
                        telemetry.addData("Tag " + tag.id, "visible; pose unavailable");
                    }
                }
                telemetry.update();
                sleep(50);
            }
        } finally {
            drive.stop();
            portal.close();
        }
    }

    private ColorBlobLocatorProcessor createColorLocator(ColorRange range) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
    }

    private List<ColorBlobLocatorProcessor.Blob> filteredBlobs(ColorBlobLocatorProcessor processor) {
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_BLOB_AREA, MAX_BLOB_AREA, blobs);
        return blobs;
    }

    private ColorBlobLocatorProcessor.Blob largest(List<ColorBlobLocatorProcessor.Blob> blobs) {
        ColorBlobLocatorProcessor.Blob best = blobs.get(0);
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (blob.getContourArea() > best.getContourArea()) {
                best = blob;
            }
        }
        return best;
    }

    private int largestArea(ColorBlobLocatorProcessor.Blob blob) {
        return blob.getContourArea();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
