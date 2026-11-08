/*
 * Hive Vision — Control Hub track (OpenCV, no model)
 * Color-threshold ball detector for FTC. Runs on the Control Hub via
 * VisionPortal (VisionProcessor), no coprocessor required.
 *
 * HSV ranges and geometry gates were learned from the Limelight track's
 * YOLO model on real match footage (fit_hsv_from_yolo.py).
 * Re-tune on your actual field camera with hsv_tuner.py and copy the
 * constants here if lighting differs.
 *
 * Metrics (vs YOLO truth on the development match video):
 *   yellow: recall 68.7%, precision 39.9%
 *   red   : recall 80.1%, precision 26.0%   (red panels/tape are FP-heavy)
 *   blue  : recall 82.3%, precision 65.5%
 *
 * Typical HSV (OpenCV H 0..180, S/V 0..255):
 *   yellow: H 9..33,   S 90..255,  V 45..238
 *   red   : H 0..15 and 170..180,  S 81..255,  V 45..232
 *   blue  : H 102..128, S 80..255, V 45..247
 */
package  org.firstinspires.ftc.teamcode.Vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // not used, but common import

public class BallDetectorPipeline implements VisionProcessor {

    /* ------------------------------------------------------------------
     * 1) HSV segments (lower/upper per color). Red is matched with two
     *    segments because its hue wraps through 0.
     * ----------------------------------------------------------------*/
    public enum BallColor { YELLOW, RED, BLUE }

    private static final Scalar YELLOW_LO = new Scalar(9, 90, 45);
    private static final Scalar YELLOW_HI = new Scalar(33, 255, 238);

    private static final Scalar RED_LO_A = new Scalar(0, 81, 45);
    private static final Scalar RED_HI_A = new Scalar(15, 255, 232);
    private static final Scalar RED_LO_B = new Scalar(170, 81, 45);
    private static final Scalar RED_HI_B = new Scalar(180, 255, 232);

    private static final Scalar BLUE_LO = new Scalar(102, 80, 45);
    private static final Scalar BLUE_HI = new Scalar(128, 255, 247);

    /* ------------------------------------------------------------------
     * 2) Shape/size gating (pixels at camera resolution).
     *    Real balls on the FTC field are small-to-mid and roughly square;
     *    robot side panels are giant or elongated. Tune for your camera.
     * ----------------------------------------------------------------*/
    private static final int   MIN_AREA_PX  = 900;    // kills speckle patches
    private static final int   MAX_AREA_PX  = 12000;  // kills big robot panels
    private static final double MIN_ASPECT  = 0.71;   // near-square balls only
    private static final double MAX_ASPECT  = 1.4;
    private static final double MIN_FILL    = 0.5;    // contourArea / rectArea
    private static final double MAX_FILL    = 1.3;
    private static final boolean USE_MORPH = true;    // close small gaps

    /** Typical real-ball contour area per color (px at camera resolution). */
    private static final double EXPECT_YELLOW_AREA = 3500;
    private static final double EXPECT_RED_AREA    = 4200;
    private static final double EXPECT_BLUE_AREA   = 4200;

    private static double expectArea(BallColor c) {
        switch (c) {
            case YELLOW: return EXPECT_YELLOW_AREA;
            case BLUE:   return EXPECT_BLUE_AREA;
            default:     return EXPECT_RED_AREA;
        }
    }

    private boolean suspend = false;
    private boolean detect  = true;
    private final Object lock = new Object();
    private List<BallBlob> detections = new ArrayList<>();

    /** One detected ball candidate for the OpMode. */
    public static class BallBlob {
        public final BallColor color;
        public final Rect   rect;
        public final double area;
        public final double cx, cy;
        public double cxNorm = 0, cyNorm = 0, wNorm = 0, hNorm = 0;

        BallBlob(BallColor c, Rect r, double a) {
            color = c;
            rect = r;
            area = a;
            cx = r.x + r.width / 2.0;
            cy = r.y + r.height / 2.0;
        }

        void normalize(int frameW, int frameH) {
            cxNorm = cx / frameW;
            cyNorm = cy / frameH;
            wNorm = rect.width / (double) frameW;
            hNorm = rect.height / (double) frameH;
        }
    }

    /** Most ball-like detected blob of a color (candidate, not ground truth). */
    public BallBlob bestOf(BallColor color) {
        BallBlob best = null;
        double bestCost = Double.MAX_VALUE;
        synchronized (lock) {
            for (BallBlob b : detections) {
                if (b.color != color) continue;
                double fill = b.area / (double) (b.rect.width * b.rect.height);
                double cost = Math.abs(b.area / expectArea(color) - 1.0)
                            + 4.0 * (1.0 - fill);
                if (cost < bestCost) {
                    bestCost = cost;
                    best = b;
                }
            }
        }
        return best;
    }

    public List<BallBlob> getDetections() {
        synchronized (lock) {
            return new ArrayList<>(detections);
        }
    }

    public void suspendDetection() { detect = false; }
    public void resumeDetection()  { detect = true; }
    public void suspend()          { suspend = true; }
    public void resume()           { suspend = false; }

    @Override
    public void init(int width, int height, org.firstinspires.ftc.vision.CameraCalibration calibration) {
        // No per-resolution init needed; constants are pixel-based.
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (suspend) {
            return input; // return unmodified
        }
        List<BallBlob> out = new ArrayList<>();
        if (detect) {
            maskAndFind(input, BallColor.YELLOW, YELLOW_LO, YELLOW_HI, null, null, out);
            maskAndFind(input, BallColor.RED,    RED_LO_A, RED_HI_A, RED_LO_B, RED_HI_B, out);
            maskAndFind(input, BallColor.BLUE,   BLUE_LO, BLUE_HI, null, null, out);
        }
        for (BallBlob b : out) b.normalize(input.cols(), input.rows());

        synchronized (lock) { detections = out; }

        // Optional: draw overlay for debug (VisionPortal can display it)
        Mat output = input.clone();
        if (detect) drawOverlay(output, out);
        return output;
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No custom canvas drawing needed.
    }

    private void maskAndFind(Mat input, BallColor color, Scalar loA, Scalar hiA,
                             Scalar loB, Scalar hiB, List<BallBlob> out) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        if (loB != null) { // red wrap: union of two masks
            Mat m2 = new Mat();
            Core.inRange(hsv, loA, hiA, mask);
            Core.inRange(hsv, loB, hiB, m2);
            Core.bitwise_or(mask, m2, mask);
            m2.release();
        } else {
            Core.inRange(hsv, loA, hiA, mask);
        }
        hsv.release();
        if (USE_MORPH) {
            Mat el = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                                                   new org.opencv.core.Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, el);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, el);
            el.release();
        }
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hier = new Mat();
        Imgproc.findContours(mask, contours, hier, Imgproc.RETR_EXTERNAL,
                             Imgproc.CHAIN_APPROX_SIMPLE);
        hier.release();
        mask.release();
        for (MatOfPoint ct : contours) {
            double area = Imgproc.contourArea(ct);
            Rect r = Imgproc.boundingRect(ct);
            ct.release();
            if (area < MIN_AREA_PX || area > MAX_AREA_PX) continue;
            if (r.width <= 0 || r.height <= 0) continue;
            double aspect = (double) r.width / r.height;
            if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;
            double fill = area / (double) (r.width * r.height);
            if (fill < MIN_FILL || fill > MAX_FILL) continue;
            out.add(new BallBlob(color, r, area));
        }
    }

    private void drawOverlay(Mat frame, List<BallBlob> blobs) {
        for (BallBlob b : blobs) {
            Scalar col = b.color == BallColor.YELLOW ? new Scalar(0, 255, 255)
                       : b.color == BallColor.RED    ? new Scalar(0, 0, 255)
                       : new Scalar(255, 0, 0);
            Imgproc.rectangle(frame, b.rect, col, 2);
            Imgproc.putText(frame, b.color.name(),
                            new Point(b.rect.x, b.rect.y - 6),
                            Imgproc.FONT_HERSHEY_PLAIN, 1.0, col, 2);
        }
    }
}
