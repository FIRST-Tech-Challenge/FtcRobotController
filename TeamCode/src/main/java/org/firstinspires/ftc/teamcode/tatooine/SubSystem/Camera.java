package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;

import java.util.List;

public class Camera {

    private double angle = 0;

    private double prevAngle = 0;

    private OpMode opMode;

    private final boolean IS_DEBUG;

    private VisionPortal portal;
    private ColorBlobLocatorProcessor blobLocatorProcessor;

    private final boolean IS_RED;

    private boolean isSpecimen;

    private double prevPostion = 0;

    private double postion = 0;




    public Camera(OpMode opMode, boolean IS_DEBUG, boolean IS_RED) {
        this.IS_DEBUG = IS_DEBUG;
        this.opMode = opMode;
        this.IS_RED = IS_RED;
        this.isSpecimen = false;
        init();
    }
    private void init() {
        blobLocatorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(IS_RED ? ColorRange.RED : ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)                        // Show contours on the Stream Preview
                .setBlurSize(5)
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(blobLocatorProcessor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public double getPostion() {
        prevAngle = angle;
        prevPostion = postion;
        List<ColorBlobLocatorProcessor.Blob> blobs =blobLocatorProcessor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.
        if (blobs.isEmpty())
        {
           return prevPostion;
        }
        ColorBlobLocatorProcessor.Blob blob = blobs.get(0);

        Point[] pt = new Point[4];
        blob.getBoxFit().points(pt);

        double d03 = Math.sqrt((pt[0].x - pt[3].x) * (pt[0].x - pt[3].x) + (pt[0].y - pt[3].y) * (pt[0].y - pt[3].y));
        double d01 = Math.sqrt((pt[0].x - pt[1].x) * (pt[0].x - pt[1].x) + (pt[0].y - pt[1].y) * (pt[0].y - pt[1].y));

        if (d03 > d01) {
            angle = Math.toDegrees(Math.atan((pt[0].y - pt[3].y) / (pt[0].x - pt[3].x)));
        }
        else {
            angle = Math.toDegrees(Math.atan((pt[0].y - pt[1].y) / (pt[0].x - pt[1].x)));
        }
        postion = (MathUtil.optimizeAngle(angle, prevAngle)/180)+0.5;
        opMode.telemetry.addData("pos",postion/5);
        opMode.telemetry.addData("ang",MathUtil.optimizeAngle(angle, prevAngle));
        opMode.telemetry.update();
        return  postion;


    }


}
