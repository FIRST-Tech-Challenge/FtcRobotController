package org.firstinspires.ftc.teamcode.Testing.Helper_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class setManualExposure extends LinearOpMode {
    doubleVision DoubleVision = new doubleVision();

    // ---------------------------------------------------------------------------------
    // Manually set the camera gain and exposure.
    // This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    // ---------------------------------------------------------------------------------
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (DoubleVision.visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (DoubleVision.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (DoubleVision.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("camera", "Ready");
            telemetry.update();
        }


        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = DoubleVision.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = DoubleVision.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            sleep(20);
        }
    } // End setManualExposure()

    @Override
    public void runOpMode() {
    }
}
