package org.firstinspires.ftc.team00000.v2.vision;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

public class ColorVisionSubsystem {

    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor processor;
    private RotatedRect currentTarget;

    public ColorVisionSubsystem(WebcamName webcam) {
        // Setup processor
        processor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .setRoi(ImageRegion.entireFrame())
                .build();

        // Setup portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(processor)
                .build();
    }

    public void update() {
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
        if (!blobs.isEmpty()) {
            currentTarget = blobs.get(0).getBoxFit(); // largest by area
        } else {
            currentTarget = null;
        }
    }

    public boolean hasTarget() {
        return currentTarget != null;
    }

    public double getAngle() {
        return hasTarget() ? currentTarget.angle : 0;
    }

    public Point getCenter() {
        return hasTarget() ? currentTarget.center : new Point(0, 0);
    }

    public double getArea() {
        return hasTarget() ? currentTarget.size.area() : 0;
    }

    public void stop() {visionPortal.close();
    }
}
