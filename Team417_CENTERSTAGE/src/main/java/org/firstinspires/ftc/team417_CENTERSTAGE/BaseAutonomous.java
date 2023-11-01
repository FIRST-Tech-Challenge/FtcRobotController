package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

abstract public class BaseAutonomous extends BaseOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();

    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    private int CAMERA_WIDTH_PIXELS = 640;
    private int CAMERA_HEIGHT_PIXELS = 480;

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initializeHardware();

        telemetry.addData("Init State", "Init Finished");

        camera.setPipeline(new PropDetectionPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Set last know encoder values
        lastEncoderFR = FR.getCurrentPosition();
        lastEncoderFL = FL.getCurrentPosition();
        lastEncoderBL = BL.getCurrentPosition();
        lastEncoderBR = BR.getCurrentPosition();

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
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

    private void stopDriving() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }
}
