package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EdgeDetectionPipeline extends OpenCvPipeline {
    // Mat objects for processing
    public Telemetry telemetry;
    private Mat blurred = new Mat();
    public Mat grayMat = new Mat();
    public Mat edges = new Mat();
    public Mat input_ = new Mat();
    public List<SampleDetection> detections = new ArrayList<>();
    public int MIN_AREA = 100000;
    public int MAX_AREA = 1000000000;
    public int cameraWidth;
    public int cameraHeight;
    public void init(int width, int height, MultipleTelemetry telemetry){
        this.telemetry = telemetry;
        this.cameraHeight = height;
        this.cameraWidth = width;
    }
    // Angle of the detected sample
    private double detectedAngle = 0;
    public double area = 0;
    public int currentId = 1;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to grayscale
        Imgproc.GaussianBlur(input, blurred, new Size(5, 5), 0);
        Imgproc.Canny(blurred, edges, 50, 150);
// Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        input.copyTo(input_);
        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            SampleDetection detection = new SampleDetection(currentId, ColorRange.RED, rect, true);
            area = rect.size.height * rect.size.width;
            // Filter by size and aspect ratio
            if (area > MIN_AREA && area < MAX_AREA) {
                detections.add(detection);
                detectedAngle = rect.angle;
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
                telemetry.addLine("\t Angle: " + closest.angle);
                blurred.empty();
                edges.empty();
            }
            detections.clear();
        }
        // Return the image with contours and angle annotations
        return input_;
    }

    public double getDetectedAngle() {
        return detectedAngle;
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
