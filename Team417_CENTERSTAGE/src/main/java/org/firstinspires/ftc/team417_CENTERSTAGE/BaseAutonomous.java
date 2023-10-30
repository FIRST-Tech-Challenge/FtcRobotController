package org.firstinspires.ftc.team417_CENTERSTAGE;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

abstract public class BaseAutonomous extends BaseOpMode {
    OpenCvCamera camera; // calls camera

    static final double FEET_PER_METER = 3.28084;
    static final int CAMERA_WIDTH_PIXELS = 800;
    static final int CAMERA_HEIGHT_PIXELS = 448;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;

    static final double HOLD_ARM_AT_MID_OR_LOW_POS_POWER = 0.005;
    static final double HOLD_ARM_AT_GROUND_POS_POWER = 0.01;
    static final double ARM_RAISE_POWER = 1.0 / 800.0;

    public void initializeAuto() {
        initializeHardware();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new PropDetectionPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    public static final Scalar LOWER_BLUE_OR_RED = new Scalar(100, 50, 50);
    public static final Scalar UPPER_BLUE_OR_RED = new Scalar(130, 255, 255);

    boolean detectingBlue;

    enum SideDetected {
        LEFT,
        CENTER,
        RIGHT
    }

    SideDetected sideDetected;

    class PropDetectionPipeline extends OpenCvPipeline {
        boolean viewportPaused = false;
        Mat hsv = new Mat();
        Mat output = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // convert image to grayscale
            if (detectingBlue) {
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            } else {
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            }
            // blur the image to reduce the impact of noisy pixels
            Imgproc.GaussianBlur(hsv, hsv, new Size(7, 7), 0);
            Core.inRange(hsv, LOWER_BLUE_OR_RED, UPPER_BLUE_OR_RED, hsv);
            Imgproc.threshold(hsv, output, 1, 255, Imgproc.THRESH_BINARY);
            // Resize the binary mask
            Mat resizedMask = new Mat();
            Imgproc.resize(output, resizedMask, new Size(0, 0), 0.7, 0.7, Imgproc.INTER_AREA);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(resizedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.size() > 0) {
                // Find the largest contour
                double maxArea = -1;
                int maxAreaIndex = -1;
                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIndex = i;
                    }
                }

                MatOfPoint largestContour = contours.get(maxAreaIndex);
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                // Draw a rectangle around the largest contour on the frame
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);

                // Calculate the center of the bounding rectangle
                Point center = new Point(boundingRect.x + (boundingRect.width * 0.5), boundingRect.y + (boundingRect.height * 0.5));

                // Draw the largest contour
                Imgproc.drawContours(input, contours, maxAreaIndex, new Scalar(0, 255, 0));

                // Draw a circle at the center
                Imgproc.circle(input, center, 2, new Scalar(0, 255, 0), 2);

                int width = input.width();
                int contourX = (int) center.x;

                // Determine position based on the X coordinate of the center
                if (contourX < width / 3) {
                    sideDetected = SideDetected.LEFT;
                } else if (contourX > (2 * width) / 3) {
                    sideDetected = SideDetected.RIGHT;
                } else {
                    sideDetected = SideDetected.CENTER;
                }
            }
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                camera.pauseViewport();
            } else {
                camera.resumeViewport();
            }
        }
    }

    public SideDetected detectTeamProp() {
        return sideDetected;
    }

    /**
     * Updates telemetry: updates the id if no april tag is sighted
     */
    public void updateTelemetryAfterStart() {
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
    }
}
