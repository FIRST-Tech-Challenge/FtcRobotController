package org.firstinspires.ftc.teamcode.CVRec;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVDetector {
    private HardwareMap hwMap;
    private OpenCvCamera webcam = null;

    private CVPipelineBase activePipeline = null;

    private static String TAG = "CVDetector";

    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);

    public CVDetector(HardwareMap hw){
        this.hwMap = hw;
    }

    public void init(CVDetectMode mode, String camID, String monitorID){

        try {
            int cameraMonitorViewId = -1;
            if (!monitorID.isEmpty()) {
                cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier(monitorID, "id", hwMap.appContext.getPackageName());
            }
            Log.d(TAG, String.format("Webcam monitor ID =  %d", cameraMonitorViewId));

            if (cameraMonitorViewId >= 0) {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, camID), cameraMonitorViewId);
            } else {
                webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, camID));
            }

            if (mode == CVDetectMode.Stack) {
                activePipeline = new CVRingStackPipeline();
            } else if (mode == CVDetectMode.Search) {
                activePipeline = new CVRingSearchPipeline();
            }
            if (webcam == null) {
                Log.d(TAG, String.format("Webcam %s cannot be initialized", camID));
            } else {
                Log.d(TAG, String.format("Webcam %s initialized", camID));
            }
            if (activePipeline != null) {
                webcam.setPipeline(activePipeline);
            }
        }
        catch (Exception ex){
            Log.e(TAG, String.format("Cannot init detector. %s", ex.toString()));
        }
    }

    public void startDetection(){
        if (webcam != null) {
            Log.d(TAG, "CV Detector opening camera ");
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    Log.d(TAG, "CV Detector camera streaming started");
                }
            });
        }
    }

    public void stopDetection(){
        if (webcam != null){
            webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override
                public void onClose() {
                    Log.d(TAG, "CV Detector camera closed");
                }
            });
        }
    }

    public RingStackSize getStackSize(){
        if (activePipeline != null){
            return activePipeline.getStackSize();
        }
        return RingStackSize.Undefined;
    }

    public int getMeanVal(){
        if (activePipeline != null){
            return activePipeline.getMeanVal();
        }
        return -1;
    }
}
