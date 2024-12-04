package org.firstinspires.ftc.teamcode.tatooine.SubSystem.camera;

import android.graphics.Canvas;

import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public abstract class BlobProcessorEX extends ColorBlobLocatorProcessor {
    double angle = 0;
    BlobSort sort = new BlobSort(BlobCriteria.BY_CONTOUR_AREA, SortOrder.ASCENDING);


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        setSort(sort);

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Blob blob = getBlobs().get(0);
        Point[] pt = new Point[4];
        blob.getBoxFit().points(pt);
        Rect bbox = blob.getBoxFit().boundingRect();
        Rect rect = new Rect(new Point(bbox.x, bbox.y + bbox.height / 2.0),
                new Point(bbox.x + bbox.width / 2.0, bbox.y + bbox.height));

        if (rect.contains(pt[0]) && rect.contains(pt[1])){
            angle = -blob.getBoxFit().angle;
        }
        else {
            angle = blob.getBoxFit().angle;
        }
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    public double getAngle() {
        return angle;
    }
}
