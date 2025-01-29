package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

@Config
public class EdgeDetectionPipeline extends OpenCvPipeline {
    // Mat objects for processing
    public Telemetry telemetry;
    private Mat blurred = new Mat();
    public Mat grayMat = new Mat();
    public Mat edges = new Mat();
    public Mat input_ = new Mat();
    public static int CANNY_THRESHOLD_1 = 50;
    public static int CANNY_THRESHOLD_2 = 150;
    public List<SampleDetection> detections = new ArrayList<>();
    public static int MIN_AREA = 0;
    public static int FRAME_TO_RETURN = 0;
    public static BigInteger MAX_AREA = new BigInteger("10000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
    public boolean convert = false;
    public int cameraWidth;
    public int cameraHeight;
    public void init(int width, int height, MultipleTelemetry telemetry, boolean convertToGrayscale){
        this.telemetry = telemetry;
        this.cameraHeight = height;
        this.cameraWidth = width;
        this.convert = convertToGrayscale;
    }
    // Angle of the detected sample
    private double detectedAngle = 0;
    public double area = 0;
    public int currentId = 1;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to grayscale
        if(convert){
            Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_BGR2GRAY);
        }
        else {
            input.copyTo(grayMat);
        }
        Imgproc.GaussianBlur(grayMat, blurred, new Size(5, 5), 0);
        Imgproc.Canny(blurred, edges, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2);
// Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        input.copyTo(input_);
        telemetry.addData("Contours Found", contours.size());
        telemetry.update();
        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            SampleDetection detection = new SampleDetection(currentId, ColorRange.RED, rect, true);
            area = rect.size.height * rect.size.width;
            // Filter by size and aspect ratio
            if (area > MIN_AREA && area < MAX_AREA.intValue()) {
                detections.add(detection);
                detectedAngle = getAngle(rect);
                currentId = currentId + 1;
                SampleDetection closest = getClosestDetection();
                Point[] vertices = new Point[4];
                closest.boxFit.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input_, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2); // Green lines
                }
                telemetry.addLine("Detected Sample:");
                telemetry.addLine("\t Width: " + closest.boxFit.size.width);
                telemetry.addLine("\t Height: " + closest.boxFit.size.height);
                telemetry.addLine("\t Angle: " + detectedAngle);
                if(FRAME_TO_RETURN == 0) {
                    blurred.empty();
                    edges.empty();
                }
                else {
                    input_.empty();
                }
            }
            detections.clear();
        }
        // Return the image with contours and angle annotations
        if(FRAME_TO_RETURN == 0){
            return input_;
        }
        else if(FRAME_TO_RETURN == 1) {
            return edges;
        }
        else {
            return blurred;
        }
    }

    public double getDetectedAngle() {
        return detectedAngle;
    }

    public double getAngle(RotatedRect boxFit) {
        double angle;
        if (boxFit.size.width < boxFit.size.height) {
            angle = boxFit.angle - 90;
        }
        else {
            angle = boxFit.angle;
        }
        return angle;
    }

    public SampleDetection getClosestDetection(){
        SampleDetection closest = null;
        double closestDistance = cameraHeight *  cameraWidth + 1000000000;
        for(SampleDetection detection : detections){
            double centerX = cameraWidth / 2.0;
            double centerY = cameraHeight / 2.0;
            double xOffset = Math.abs(detection.boxFit.center.x - centerX);
            double yOffset = Math.abs(detection.boxFit.center.y - centerY);
            Point point = new Point(centerX, centerY);  // Example: center of the image
            // Define the color (BGR format) and thickness
            Scalar color = new Scalar(0, 255, 0);  // Green color
            int thickness = 2;

            // Draw the point (a small filled circle)
            Imgproc.circle(input_, point, 2, color, thickness);
            if(closestDistance > (xOffset + yOffset)){
                closestDistance = xOffset + yOffset;
                closest = detection;
            }
        }
        if(closest == null){
            closest = new SampleDetection(-1, ColorRange.YELLOW, new RotatedRect(), false);
        }
        return closest;
    }
}
