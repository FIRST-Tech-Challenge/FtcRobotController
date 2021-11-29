package org.firstinspires.ftc.teamcode.core.robot.vision.robot;

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
    private final EventThread eventThread;
    private TsePipeline pipeline;
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;

    public TseDetector(@NonNull EventThread eventThread, HardwareMap hMap, String webcamName) {
        this.eventThread = eventThread;
        this.hardwareMap = hMap;
        this.webcamName = webcamName;
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

    /**
     * Resets pipeline on call
     * Stalls code until pipeline is done with figuring out (max time of around 0.33 seconds)
     * @return integer 1 - 3, corresponds to barcode slots left to right
     */
    public synchronized int run() {
        pipeline.startPipeline();
        boolean first = pipeline.differentSpot().first;
        while (!first) {
            first = pipeline.differentSpot().first;
        }
        pipeline.stopPipeline();
        return pipeline.differentSpot().second;
    }
}
