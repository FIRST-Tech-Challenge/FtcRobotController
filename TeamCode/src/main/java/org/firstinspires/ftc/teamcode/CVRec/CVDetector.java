package org.firstinspires.ftc.teamcode.CVRec;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public class CVDetector {
    private HardwareMap hwMap;
    private OpenCvCamera webcam = null;
    private int resX = 320;
    private int resY = 240;
    private String opModeSide = AutoRoute.NAME_RED;

    private CVPipelineBase activePipeline = null;

    private static String TAG = "CVDetector";

    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar YELLOW = new Scalar(255, 255, 0);

    private ArrayList<AutoDot> namedCoordinates = new ArrayList<>();

    private static AutoDot levelA = new AutoDot("A", 75, 75, -1, AutoRoute.NAME_RED);
    private static AutoDot levelB = new AutoDot("B", 65, 100, -1, AutoRoute.NAME_RED);
    private static AutoDot levelC = new AutoDot("C", 75, 120, -1, AutoRoute.NAME_RED);

    public CVDetector(HardwareMap hw, String side, CVDetectMode mode){
        this(hw, side, mode, null);
    }

    public CVDetector(HardwareMap hw, String side, CVDetectMode mode, ArrayList<AutoDot> namedCoordinates){
        this.hwMap = hw;
        this.opModeSide = side;
        if (namedCoordinates != null) {
            this.namedCoordinates = namedCoordinates;
        }
        configLevels(this.opModeSide);
        init(mode, "wcam", "cameraMonitorViewId");
    }

    protected void configLevels(String side){
        if (this.namedCoordinates.size() > 0){
            for(AutoDot d : namedCoordinates){
                if (d.getFieldSide().equals(side)) {
                    if (d.getFieldSide().equals(side)) {
                        if (d.getDotName().equals("A")) {
                            levelA = d;
                        } else if (d.getDotName().equals("B")) {
                            levelB = d;
                        } else if (d.getDotName().equals("C")) {
                            levelC = d;
                        }
                    }
                }
            }
        }
    }

    private void init(CVDetectMode mode, String camID, String monitorID){

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
                activePipeline = new CVRingStackPipeline(resX, resY);
            } else if (mode == CVDetectMode.Search) {
                activePipeline = new CVRingSearchPipeline(resX, resY);
            } else if (mode == CVDetectMode.Frenzy) {
                activePipeline = new CVFrenzyPipeline(resX, resY);
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
                    try {
                        webcam.startStreaming(resX, resY, OpenCvCameraRotation.UPRIGHT);
                        Log.d(TAG, "CV Detector camera streaming started");
                    }
                    catch (Exception ex){
                        Log.e(TAG, String.format("Cannot start streaming. %s", ex.toString()));
                    }
                }

                @Override
                public void onError(int errorCode) {
                    Log.e(TAG, String.format("Cannot open camera for streaming. Code  %d", errorCode));
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

    public GameElement getGameElement(){
        if (activePipeline != null){
            return activePipeline.getGameElement();
        }
        return GameElement.BarcodeLevel3;
    }

    public AutoDot getLevel(){
        AutoDot level = null;
        if (activePipeline != null){
            switch (activePipeline.getGameElement()){
                case BarcodeLevel1:
                    level = levelA;
                    break;
                case BarcodeLevel2:
                    level = levelB;
                    break;
                case BarcodeLevel3:
                    level = levelC;
                    break;
            }
        }
        return level;
    }

    public int getMeanVal(){
        if (activePipeline != null){
            return activePipeline.getMeanVal();
        }
        return -1;
    }

    public List<CVRoi> getTargets() {
        if (activePipeline != null){
            return activePipeline.getTargets();
        }
        return null;
    }

    public CVRoi getNearestTarget() {
        if (activePipeline != null){
            return activePipeline.getNearestTarget();
        }
        return null;
    }

    public CVRoi getSecondTarget() {
        if (activePipeline != null){
            return activePipeline.getSecondTarget();
        }
        return null;
    }

    public void resetTargets() {
        if (activePipeline != null){
            activePipeline.clearTargets();
        }
    }
}
