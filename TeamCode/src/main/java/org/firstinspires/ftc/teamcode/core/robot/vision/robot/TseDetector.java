package org.firstinspires.ftc.teamcode.core.robot.vision.robot;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.thread.thread.EventThread;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TseDetector {

    private OpenCvCamera camera;
    private boolean isUsingWebcam;
    private String webcamName;
    private final HardwareMap hardwareMap;
    private TsePipeline pipeline;
    private EventThread eventThread;
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;

    private boolean firstValue = true;
    private long lastFrameCount = 0;
    private int checks = 0;
    private Pair<Integer, Integer> greatestConfidence = new Pair<>(0, 0);
    private int lastFrameValue = 0;
    public TseDetector(EventThread eventThread, HardwareMap hMap, String webcamName) {
        this.eventThread = eventThread;
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
    }

    public void init() {
        int cameraMonitorViewId = hardwareMap
                .appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        camera.setPipeline(pipeline = new TsePipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public int process() {
        while (!Thread.currentThread().isInterrupted()) {
            if (pipeline.getFramecount() != lastFrameCount) {
                lastFrameCount++;
                int frameValue = pipeline.getDifferent();
                if (frameValue == lastFrameValue || firstValue) {
                    firstValue = false;
                    checks++;
                } else {
                    if (greatestConfidence.second < checks) {
                        greatestConfidence = new Pair<>(frameValue, checks);
                    }
                    checks = 0;
                    firstValue = true;
                }
                lastFrameValue = frameValue;
            }
            if (checks >= 5) {
                return lastFrameValue;
            }
        }
        return greatestConfidence.first;
    }
    
}
