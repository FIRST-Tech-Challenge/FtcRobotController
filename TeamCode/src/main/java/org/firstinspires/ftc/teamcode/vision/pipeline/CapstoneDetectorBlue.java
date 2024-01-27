package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.tests.VisionTestBlue;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CapstoneDetectorBlue {

    private OpenCvCamera camera;
    private final String cameraName;
    private VisionTestBlue capstonePipeline;
    private final HardwareMap hardwareMap;
    private final int width, height;

    public CapstoneDetectorBlue(HardwareMap hMap, String camName) {
        hardwareMap = hMap;
        cameraName = camName;
        width = 432;
        height = 240;
    }

    public CapstoneDetectorBlue(HardwareMap hMap, String camName, int width, int height) {
        hardwareMap = hMap;
        cameraName = camName;
        this.width = width;
        this.height = height;
    }

    public void init() {
        int cameraViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraViewId);

        capstonePipeline = new VisionTestBlue();

        camera.setPipeline(capstonePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.addGlobalWarningMessage("Warning: Camera device failed to open with EasyOpenCv error: " +
                        ((errorCode == -1) ? "CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE" : "CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE"));
            }
        });
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public void setLowerBound(Scalar low) {
        capstonePipeline.setLowerBound(low);
    }

    public void setUpperBound(Scalar high) {
        capstonePipeline.setUpperBound(high);
    }

    public void setLowerAndUpperBounds(Scalar low, Scalar high) {
        capstonePipeline.setLowerAndUpperBounds(low, high);
    }

    //Todo: tune these values they are just estimations for now
    public Placement getPlacement() {
        if (capstonePipeline.getCentroid() != null) {
            if (capstonePipeline.getCentroid().x > width - (width / 3.2))
                return Placement.RIGHT;
            else if (capstonePipeline.getCentroid().x < width - (width / 1.6))
                return Placement.LEFT;
        }
        return Placement.CENTER;
    }

    public enum Placement {
        LEFT,
        RIGHT,
        CENTER
    }

}