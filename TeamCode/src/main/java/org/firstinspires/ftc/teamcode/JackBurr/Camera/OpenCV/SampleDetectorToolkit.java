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
            master.add(new SampleDetection(color, blob.getBoxFit()));
        }
        return master;
    }
}
