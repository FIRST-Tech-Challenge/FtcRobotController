package org.firstinspires.ftc.teamcode.vision.dogecv;

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
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import java.util.List;
import java.util.concurrent.BlockingQueue;

public class DogeCVIntegration implements VisionProvider {

    private static final DogeCVFinalStep FINAL_STEP = DogeCVFinalStep.THREE_MINERALS;

    private VuforiaLocalizer vuforia;
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> q;
    private int state = -1;
    private Mat mat, display;
    private List<MatOfPoint> contours;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    private boolean enableTelemetry;
    private DogeCVPipeline pipeline;

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
        pipeline = new DogeCVPipeline();
    }

    @Override
    public void shutdownVision() {}

    @Override
    public GoldPos detect() {
        telemetry.addData("DogeCV State", state);
        switch (state) {
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
                display = pipeline.process(mat, FINAL_STEP);
                break;
            case 2:
                if(!enableTelemetry)
                    break;
                Bitmap displayBitmap = Bitmap.createBitmap(display.width(), display.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(display, displayBitmap);
                dashboard.sendImage(displayBitmap);
                display.release();
                break;
            case 3:
                state = 0;
                return pipeline.getCurrentOrder();
        }
        state++;
        return GoldPos.HOLD_STATE;
    }

    @Override
    public void reset() {
        state = 0;
    }
}
