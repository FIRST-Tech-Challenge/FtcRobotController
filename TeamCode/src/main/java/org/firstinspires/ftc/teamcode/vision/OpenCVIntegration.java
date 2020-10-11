package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.BlockingQueue;

public class OpenCVIntegration implements VisionProvider {

    private VuforiaLocalizer vuforia;
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> q;
    private int state = -1;
    private Mat mat;
    private List<MatOfPoint> contours;
    private Point lowest;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    private RoverRuckusGripPipeline pipeline;
    private boolean enableTelemetry;

    private int _numbefOfContours = -9999;

    private void initVuforia(HardwareMap hardwareMap, Viewpoint viewpoint) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        if (viewpoint == Viewpoint.BACK)
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        else if (viewpoint == Viewpoint.WEBCAM)
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        else
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
    }

    @Override
    public void initializeVision(HardwareMap hardwareMap, Telemetry telemetry, boolean enableTelemetry, Viewpoint viewpoint) {
        initVuforia(hardwareMap, viewpoint);
        q = vuforia.getFrameQueue();
        state = 0;
        this.telemetry = telemetry;
        this.enableTelemetry = enableTelemetry;
        if(enableTelemetry)
            dashboard = FtcDashboard.getInstance();
        pipeline = new RoverRuckusGripPipeline();
    }

    @Override
    public void shutdownVision() {}

    @Override
    public void reset() {
        state = 0;
    }

    @Override
    public GoldPos detect() {
        switch(state) {
            case 0:
                if (q.isEmpty())
                    return GoldPos.HOLD_STATE;
                VuforiaLocalizer.CloseableFrame frame;
                try {
                    frame = q.take();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                Image img = VisionUtils.getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(img.getPixels());
                mat = VisionUtils.bitmapToMat(bm, CvType.CV_8UC3);
                break;
            case 1:
                pipeline.process(mat);
                mat.release();
                contours = pipeline.filterContoursOutput();
                _numbefOfContours = contours.size();
                break;
            case 2:
                if(!enableTelemetry)
                    break;
                Mat overlay = pipeline.normalizeOutput().clone();
                for (int i = 0; i < contours.size(); i++) {
                    Imgproc.drawContours(overlay, contours, i, new Scalar(Math.random()*255, Math.random()*255, Math.random()*255), 2);
                }
                Bitmap overlayBitmap = Bitmap.createBitmap(overlay.width(), overlay.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(overlay, overlayBitmap);
                dashboard.sendImage(overlayBitmap);
                overlay.release();
                break;
            case 3:
                pipeline.resizeImageOutput().release();
                pipeline.normalizeOutput().release();
                pipeline.hsvThresholdOutput().release();
                if (contours.size() == 0) {
                    state = -2;
                    return GoldPos.NONE_FOUND;
                }
                lowest = null;
                for (MatOfPoint contour : contours) {
                    Point centroid = centroidish(contour);
                    if (lowest.y > centroid.y)
                        lowest = centroid;
                }
                break;
            case 4:
                for (MatOfPoint contour : pipeline.findContoursOutput())
                    contour.release();
                for (MatOfPoint contour : pipeline.filterContoursOutput())
                    contour.release();
                state = 0;
                if (lowest.x < 320d / 3)
                    return GoldPos.LEFT;
                else if (lowest.x < 640d / 3)
                    return GoldPos.MIDDLE;
                else
                    return GoldPos.RIGHT;
            default:
                return GoldPos.ERROR2;
        }
        state++;
        telemetry.addData("OpenCV State Machine State", state);
        telemetry.addData("OpenCV # of contours", _numbefOfContours);
        return GoldPos.HOLD_STATE;
    }

    private static Point centroidish(MatOfPoint matOfPoint) {
        Rect br = Imgproc.boundingRect(matOfPoint);
        return new Point(br.x + br.width/2,br.y + br.height/2);
    }
}
