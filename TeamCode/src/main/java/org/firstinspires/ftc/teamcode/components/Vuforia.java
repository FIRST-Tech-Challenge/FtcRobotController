package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
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

    private static final String VUFORIA_KEY = VuforiaParams.VUFORIA_KEY;

    private static final float mmPerInch      = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;  // the height of the center of the target image above the floor
    private static final float stoneZ         = 2.00f * mmPerInch;
    private static final float bridgeZ        = 6.42f * mmPerInch;
    private static final float bridgeY        = 23 * mmPerInch;
    private static final float bridgeX        = 5.18f * mmPerInch;
    private static final float bridgeRotY     = 59;                    // Units are degrees
    private static final float bridgeRotZ     = 180;
    private static final float halfField      = 72 * mmPerInch;
    private static final float quadField      = 36 * mmPerInch;
    private OpenGLMatrix lastLocation         = null;
    private VuforiaLocalizer vuforia          = null;
    public VuforiaTrackables targetsPictures;
    private List<VuforiaTrackable> allTrackables;

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
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                if (trackable.getName().equals(targetTrackable.getName())) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
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
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                return true;
            }
        }
        return false;
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

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
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
        initializeTrackables(vuforia, "Power Play");
        return vuforia;
    }

    private void initializeTrackables(VuforiaLocalizer vuforia, String game) {
        //load the trackable pictures, TODO change the game name
        targetsPictures = vuforia.loadTrackablesFromAsset(game);



        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsPictures);



        //Set the position of the perimeter targets with relation to origin (center of field)
        //4 target that stay constant each year, add more if needed
//        red1.setLocation(OpenGLMatrix
//                .translation(quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        red2.setLocation(OpenGLMatrix
//                .translation(-quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        front1.setLocation(OpenGLMatrix
//                .translation(-halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        front2.setLocation(OpenGLMatrix
//                .translation(-halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blue1.setLocation(OpenGLMatrix
//                .translation(-quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        blue2.setLocation(OpenGLMatrix
//                .translation(quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        rear1.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//
//        rear2.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // TODO most likely will need to end up establishing precise positions in the future
        /**  Let all the trackable listeners know where the phone is.  */
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        targetsPictures.activate();
    }
}
