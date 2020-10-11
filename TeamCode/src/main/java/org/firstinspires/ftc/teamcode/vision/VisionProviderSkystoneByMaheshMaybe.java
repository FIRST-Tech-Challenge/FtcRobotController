package org.firstinspires.ftc.teamcode.vision;

//import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaSkyStoneNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.samples.ConceptVuforiaSkyStoneNavigationWebcam;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VisionProviderSkystoneByMaheshMaybe {

    private int posState;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
    private OpenGLMatrix lastLocation;

    private static final float mmPerInch = 25.4f;


    public VisionProviderSkystoneByMaheshMaybe(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, 2.00f * mmPerInch)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 0, 0, 0));

        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, VuforiaLocalizer.CameraDirection.FRONT);
    }

    public void reset() {
    }

    public double detect() {

        boolean targetVisible = false;
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            float x = translation.get(0) / mmPerInch;

            return x;
        }
        else {
            return 0;
        }
    }

    public void shutdownVision() {
    }
}
