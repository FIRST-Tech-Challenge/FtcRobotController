package org.firstinspires.ftc.team12397.v2.cameraSoftware;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team12397.v2.cameraSoftware.util.ExponentialMovingAverage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

@Config
public class ColorVisionSubsystem {

    /* -------- dashboard‑tunable blob filters -------- */
    public static int        MIN_AREA_PX      =  50;
    public static int        MAX_AREA_PX      = 20000;
    public static double     MIN_ASPECT_RATIO = 1.25;
    public static ColorRange TARGET_COLOR     = ColorRange.BLUE;
    public static double     ANGLE_SMOOTHING_ALPHA = 0.20;

    private final VisionPortal portal;
    private final ColorBlobLocatorProcessor proc;
    private final ExponentialMovingAverage  angleEma =
            new ExponentialMovingAverage(ANGLE_SMOOTHING_ALPHA);

    private RotatedRect target;

    public ColorVisionSubsystem(WebcamName cam) {
        proc = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(TARGET_COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .setRoi(ImageRegion.entireFrame())
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(cam)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(proc)
                .build();
    }

    public void update() {

        List<ColorBlobLocatorProcessor.Blob> blobs = proc.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_AREA_PX, MAX_AREA_PX, blobs);

        target = null;                                           // pessimistic default
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            RotatedRect r = b.getBoxFit();
            double ar     = Math.max(r.size.width, r.size.height) /
                    Math.min(r.size.width, r.size.height);
            if (ar < MIN_ASPECT_RATIO) continue;

            double angle = (r.size.width < r.size.height)
                    ? r.angle + 90
                    : r.angle;
            angle = (angle < 0) ? angle + 180 : angle;

            double smoothAngle = angleEma.update(angle);
            target = new RotatedRect(r.center, r.size, smoothAngle);
            break;                                               // take first valid blob
        }
    }

    /* convenience accessors */
    public boolean hasTarget()                 { return target != null; }
    public double  getAngle()                  { return hasTarget() ? target.angle         : 0; }
    public double  getAngleErrorToVertical()   { return getAngle() - 90; }
    public Point   getCenter()                 { return hasTarget() ? target.center        : new Point(0,0); }
    public double  getArea()                   { return hasTarget() ? target.size.area()   : 0; }
    public RotatedRect getTargetRect()         { return target; }
    public VisionPortal getPortal()            { return portal; }
    public void stop()                         { portal.close(); }
}