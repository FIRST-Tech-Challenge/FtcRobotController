package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class SampleDetectorToolkit {
    public HardwareMap hardwareMap;
    public SampleDetectorVisionPortalToolkit visionToolkit;
    public SampleDetectorToolkit (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void init(){
        this.visionToolkit = new SampleDetectorVisionPortalToolkit(hardwareMap);
    }

    public ColorBlobLocatorProcessor getNewProcessor(ColorRange range, int minArea, int maxArea) {
        init();
        return visionToolkit.createNewProcessor(range, minArea, maxArea);
    }

    public List<ColorBlobLocatorProcessor.Blob> filterByArea(int minArea, int maxArea, List<ColorBlobLocatorProcessor.Blob> blobs) {
        ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
        return blobs;
    }

    public RotatedRect getBoxFit(ColorBlobLocatorProcessor.Blob blob) {
        return blob.getBoxFit();
    }

    public double getAngle(RotatedRect boxFit) {
        return boxFit.angle;
    }

    public String getColorName(SampleDetection detection){
        if(detection.color == ColorRange.RED){
            return "RED";
        }
        else if(detection.color == ColorRange.BLUE){
            return "BLUE";
        }
        else {
            return "YELLOW";
        }
    }

    public Point getSampleCenter(RotatedRect boxFit) {
        return boxFit.center;
    }

    public String findNeededRotation(RotatedRect boxFit){
        double degrees = findNeededRotationDegrees(boxFit);
        if(degrees < 0){
            return "Rotate left " + degrees + "º";
        }
        else {
           return "Rotate right " + degrees + "º";
        }
    }

    public double findNeededRotationDegrees(RotatedRect boxFit){
        return boxFit.angle;
    }

    public Point getCenter(int width, int height) {
        width = width / 2;
        height = height / 2;
        return new Point(width, height);
    }

    public List<SampleDetection> addToSampleDetectionList(List<SampleDetection> master, ColorRange color, List<ColorBlobLocatorProcessor.Blob> blobs) {
        if(blobs.isEmpty()){
            master.add((new SampleDetection(color, new RotatedRect(), false)));
        }
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            master.add(new SampleDetection(color, blob.getBoxFit(), true));
        }
        return master;
    }

    public SampleDetection findClosestSample(Point center, List<SampleDetection> samplesList){
        double CENTER_X = center.x;
        double CENTER_Y = center.y;
        SampleDetection closestSample = null;
        double CLOSEST_DISTANCE = 9999999999999999999999999.9;
        for (SampleDetection sample : samplesList){
            if(closestSample == null){
                closestSample = sample;
            }
            double X_OFFSET = Math.abs(sample.x - CENTER_X);
            double Y_OFFSET = Math.abs(sample.y - CENTER_Y);
            double THIS_DISTANCE = X_OFFSET + Y_OFFSET;
            if(CLOSEST_DISTANCE > THIS_DISTANCE){
                CLOSEST_DISTANCE = THIS_DISTANCE;
                closestSample = sample;
            }
        }
        return closestSample;
    }
    public void putText(Mat mat, SampleDetection detection){
        Imgproc.putText(
                mat,
                String.format("Angle: %.2f", detection.angle),
                detection.boxFit.center,
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,
                new Scalar(255, 0, 0),
                2
        );
    }
}
