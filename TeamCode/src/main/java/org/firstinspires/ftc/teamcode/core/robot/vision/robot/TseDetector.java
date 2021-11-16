package org.firstinspires.ftc.teamcode.core.robot.vision.robot;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedOnceEvent;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TseDetector {

    private OpenCvCamera camera;
    private final String webcamName;
    private final HardwareMap hardwareMap;
    private TsePipeline pipeline;
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;
    private final boolean complete = false;

    public TseDetector(@NonNull EventThread eventThread, HardwareMap hMap, String webcamName) {
        hardwareMap = hMap;
        this.webcamName = webcamName;
        eventThread.addEvent(new RunWhenOutputChangedOnceEvent(this::notifyAll, () -> pipeline.differentSpot().first));
    }

    public synchronized int run() {
        try {
            wait();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return pipeline.differentSpot().second;
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
            public void onError(int errorCode) {
                System.out.println("OpenCv Pipeline error with error code " + errorCode);
            }
        });
    }

    public Pair<Boolean, Integer> getDifference() {
        return pipeline.differentSpot();
    }

}
