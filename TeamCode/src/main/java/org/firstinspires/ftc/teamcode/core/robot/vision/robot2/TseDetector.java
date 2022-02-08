package org.firstinspires.ftc.teamcode.core.robot.vision.robot2;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TseDetector {

    private final OpenCvCamera camera;
    private final String webcamName;
    private final HardwareMap hardwareMap;
    private final TsePipeline pipeline;
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;
    public TseDetector(HardwareMap hMap, String webcamName, boolean debug, boolean isRed) {
        this.hardwareMap = hMap;
        this.webcamName = webcamName;
        OpenCvCameraFactory cameraFactory = OpenCvCameraFactory.getInstance();
        if (debug) {
            int cameraMonitorViewId = hardwareMap
                    .appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            camera = cameraFactory.createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId); //for configurating remove isred from here
        } else {
            camera = cameraFactory.createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        }
        camera.setPipeline(pipeline = new TsePipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
                if (debug) {
                    FtcDashboard.getInstance().startCameraStream(camera, 10);
                }
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
     *
     * @return integer 1 - 3, corresponds to barcode slots left to right
     */
    public int run() {
        pipeline.saveImage();
        return 0;
    }
}
