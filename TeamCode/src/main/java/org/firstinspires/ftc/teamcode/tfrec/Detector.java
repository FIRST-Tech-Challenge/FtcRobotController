package org.firstinspires.ftc.teamcode.tfrec;

import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Detector implements Camera.PreviewCallback {
    private Bitmap rgbFrameBitmap = null;
    String cameraId;
    Telemetry telemetry;
    boolean useCamera2API = false;
    public void init(Context ctx, Telemetry t){
        telemetry = t;
        final CameraManager manager = (CameraManager) ctx.getSystemService(Context.CAMERA_SERVICE);
        try {
            for (final String cameraId : manager.getCameraIdList()) {
                final CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);

                final Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (facing != null && facing == CameraCharacteristics.LENS_FACING_EXTERNAL) {

                    final StreamConfigurationMap map =
                            characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);

                    if (map == null) {
                        continue;
                    }

                    // Fallback to camera1 API for internal cameras that don't have full support.
                    // This should help with legacy situations where using the camera2 API causes
                    // distorted or otherwise broken previews.
                    useCamera2API =
                            (facing == CameraCharacteristics.LENS_FACING_EXTERNAL)
                                    || isHardwareLevelSupported(
                                    characteristics, CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL);
                    telemetry.addData("Camera API lv2?: %s", useCamera2API);

                    break;
                }
            }
        } catch (CameraAccessException e) {
            telemetry.addData("Error", "Not allowed to access camera", e.getMessage());
        }
    }

    private boolean isHardwareLevelSupported(
            CameraCharacteristics characteristics, int requiredLevel) {
        int deviceLevel = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL);
        if (deviceLevel == CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY) {
            return requiredLevel == deviceLevel;
        }
        // deviceLevel is not LEGACY, can use numerical sort
        return requiredLevel <= deviceLevel;
    }

    @Override
    public void onPreviewFrame(byte[] bytes, Camera camera) {

    }
}
