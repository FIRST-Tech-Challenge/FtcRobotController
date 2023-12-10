package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.subsystems.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.Utilities;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;

public class RobotVision {

    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera

    OpenCvWebcam webcam;
    ColorDetectionPipeline cdPipeline;

    public RobotVision(boolean align_right) {

        Utilities utilities = Utilities.getSharedUtility();
        HardwareMap hardwareMap = utilities.getSharedUtility().hardwareMap;
        int cameraMonitorViewId =  hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Log.v("vision", "RobotVision constructed. align_right = " + align_right);
        cdPipeline = new ColorDetectionPipeline(align_right);
        webcam.setPipeline(cdPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                Utilities utilities = Utilities.getSharedUtility();
                utilities.telemetry.addData("Camera Failed","");
                utilities.telemetry.update();
            }
        });
    }

    public int getTeamPropOrientation(boolean isred, boolean align_right)
    {
        cdPipeline.align_right = align_right;
        return cdPipeline.getTeamPropOrientation(isred);
    }

    public void stopDetection()
    {
        cdPipeline.stop();
    }
}