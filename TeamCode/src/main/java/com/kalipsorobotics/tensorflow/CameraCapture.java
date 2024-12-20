package com.kalipsorobotics.tensorflow;

import android.os.SystemClock;
import android.util.Log;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

public class CameraCapture {
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    final boolean USING_WEBCAM = false;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime = 0;
    VisionPortal portal;

    public CameraCapture() {
        portal = new VisionPortal.Builder()
                .setCamera(INTERNAL_CAM_DIR)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
    }

    public void capture() {
        if (capReqTime == 0) {
            portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
            Log.d("Camera", "writing to CameraFrameCapture. frame count: " + frameCount);
            capReqTime = System.currentTimeMillis();
        }

        if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 500)
        {
            capReqTime = 0;
        }
    }
}
