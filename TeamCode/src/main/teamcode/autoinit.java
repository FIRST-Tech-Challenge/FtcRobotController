package org.firstinspires.ftc.teamcode;

//import android.opengl.Visibility;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class autoinit extends New_Super_Class {
    public Drive drive;

    public VisionPipeline vision;
    OpenCvCamera webcam;

    @Override
    public void inits() {
        drive = new Drive(this);
        vision = new VisionPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(vision);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while(!isStarted() && !isStopRequested() && !opModeIsActive()) {

        }
    }
}
