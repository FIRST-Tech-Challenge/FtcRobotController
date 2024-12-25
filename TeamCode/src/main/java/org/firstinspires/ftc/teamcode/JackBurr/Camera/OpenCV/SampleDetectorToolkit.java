package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectorToolkit {
    public SampleDetectorVisionPortalToolkit visionToolkit;

    public ColorBlobLocatorProcessor getNewProcessor(ColorRange range) {
        return visionToolkit.createNewProcessor(range);
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

    public Point getSampleCenter(RotatedRect boxFit) {
        return boxFit.center;
    }

    public Point getCenter(int width, int height) {
        width = width / 2;
        height = height / 2;
        return new Point(width, height);
    }

    public List<SampleDetection> addToSampleDetectionList(List<SampleDetection> master, ColorRange color, List<ColorBlobLocatorProcessor.Blob> blobs) {
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
}
