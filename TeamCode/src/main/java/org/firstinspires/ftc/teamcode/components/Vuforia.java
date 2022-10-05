package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.params.VuforiaParams;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class Vuforia {

    // Which camera to use
    public enum CameraChoice {
        PHONE_FRONT, PHONE_BACK, WEBCAM1, WEBCAM2;
    }

    private class VuforiaLocalizer extends VuforiaLocalizerImpl {
        public VuforiaLocalizer(Parameters parameters) {
            super(parameters);
        }

        public void close() {
            super.close();
        }
    }


    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;  // the height of the center of the target image above the floor
    private static final float stoneZ = 2.00f * mmPerInch;
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                    // Units are degrees
    private static final float bridgeRotZ = 180;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    public VuforiaTrackables targetsPictures;
    public VuforiaTrackables targetsAsset;
    private List<VuforiaTrackable> allTrackables;
    private List<VuforiaTrackable> teamAssetTrackables;

    public Vuforia(HardwareMap hardwareMap, CameraChoice choice) {
        vuforia = setCamera(hardwareMap, choice);
    }

    public void close() {
        vuforia.close();
    }

    public Orientation getRobotHeading() {
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
    }

    public VectorF getRobotPosition() {
        return lastLocation.getTranslation();
    }

    public boolean isTargetVisible(VuforiaTrackable targetTrackable) {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                if (trackable.getName().equals(targetTrackable.getName())) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    return true;
                }
            }
        }
        return false;
    }

    public boolean isAnyTargetVisible() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                return true;
            }
        }
        return false;
    }

    public String nameOfTargetVisible() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return trackable.getName();
            }
        }
        return "None.";
    }

    public void activate() {
        targetsPictures.activate();
    }

    public void disable() {
        targetsPictures.deactivate();
    }

    public VuforiaLocalizer setCamera(HardwareMap hardwareMap, CameraChoice cameraChoice) {
        if (vuforia != null)
            vuforia.close();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.useExtendedTracking = false;

        parameters.vuforiaLicenseKey = BuildConfig.VUFORIA_KEY;
        switch (cameraChoice) {
            case PHONE_FRONT:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                break;
            case PHONE_BACK:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                break;
            case WEBCAM1:
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
                break;
            case WEBCAM2:
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");
                break;
        }
        vuforia = new VuforiaLocalizer(parameters);
        initializeTrackables(vuforia, "PowerPlay", "FTCPowerPlay");

        return vuforia;
    }

    private void initializeTrackables(VuforiaLocalizer vuforia, String game, String asset) {
        //load the trackable pictures, TODO change the game name
        targetsPictures = vuforia.loadTrackablesFromAsset(game);
        targetsAsset = vuforia.loadTrackablesFromAsset(asset);
        // load the trackable from team code asset

        teamAssetTrackables = new ArrayList<VuforiaTrackable>();
        teamAssetTrackables.addAll(targetsAsset);


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsPictures);




        // TODO most likely will need to end up establishing precise positions in the future
        /**  Let all the trackable listeners know where the phone is.  */
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }

//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

            targetsPictures.activate();
            targetsAsset.activate();
        }


    }
    public void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targetsPictures.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public int identifyTeamAsset(){
        int i = 0;
        for (VuforiaTrackable trackable : teamAssetTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return i;
            }
            i++;
        }
        return -1;
    }

    public boolean isTeamAssetVisible(){
        for (VuforiaTrackable trackable : teamAssetTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return true;
            }
        }
        return false;
    }

}
