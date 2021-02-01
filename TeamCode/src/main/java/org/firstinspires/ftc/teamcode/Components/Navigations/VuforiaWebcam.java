/**
 * Vuforia Class. This is for detecting the
 * VuMarks. This class is for using the Webcams
 * It should display the target
 * seen, the location of the phone/robot
 * relative to the target, and the rotation/
 * angle of the phone in telemetry.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-September-30
 * @status: Fully working
 */

package org.firstinspires.ftc.teamcode.Components.Navigations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.key;
import static org.firstinspires.ftc.teamcode.Components.Navigations.Navigation.*;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VuforiaWebcam extends Thread {
    private OpMode op;
    private static double xpos, ypos, angle;
    private double vuforiaAngle = 90.0;

    //It is used on line 193 for debugging purposes. This is why it isn't currently in use.
    private String trackable;

    private static boolean targetVisible = false;

    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private OpenGLMatrix lastLocation = null;

    private final float mmPerInch = 25.4f;

    private VuforiaLocalizer vuforia = null;

    public VuforiaWebcam(OpMode opMode) {
        op = opMode;

        // Vuforia License Key
        final String VUFORIA_KEY = key.key;

        // Initialize Variables
        boolean targetVisible = false;
        final float halfField = 72 * mmPerInch;
        final float quarterField = 36 * mmPerInch;
        final float mmTargetHeight = 5.75f * mmPerInch;

        WebcamName webcamName = op.hardwareMap.get(WebcamName.class, "WebcamSide");

        // Show camera view on RC phone screen
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set parameters
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        // Start Vuforia
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        allTrackables.addAll(targetsUltimateGoal);

        // Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quarterField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quarterField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Set Webcam Location
        final float CAMERA_FORWARD_DISPLACEMENT = -1.5f;//5.5
        final float CAMERA_VERTICAL_DISPLACEMENT = 8f;//10
        final float CAMERA_LEFT_DISPLACEMENT = 0.0f;//7.25

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 180, -90, 0)); //YZX 0, 90, -90 //XYZ 0, -90, 0

        // Give Phone Location to Trackable Listeners
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate Vuforia Tracking
        targetsUltimateGoal.activate();
    }

    @Override
    public void run() {
        // Run until Thread is Interrupted
        while (!isInterrupted()) {
            targetVisible = false;
            // Look for Trackable, Update Robot Location if Possible
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                   // op.telemetry.addData("Visible Target ", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Return Location Data (Last Known Location)
            if (targetVisible) {
                setInVuforia(true);
                VectorF translation = lastLocation.getTranslation();
                //op.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        //translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                if(Math.sqrt(Math.pow(VuforiaWebcam.getVuforiaX(), 2) + Math.pow(VuforiaWebcam.getVuforiaY(), 2))>=24.5 && VuforiaWebcam.isTargetVisible()==true) {
                    setXposition(translation.get(0) / mmPerInch);
                    setYposition((translation.get(1) / mmPerInch));
                }
                xpos = translation.get(0) / mmPerInch;
                ypos = translation.get(1) / mmPerInch;
                angle = rotation.thirdAngle;
                setInVuforia(false);
                //op.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                //op.telemetry.addData("Visible Target", "none");
            }
        }
    }

    public static double getVuforiaX() {
        return xpos;
    }

    public static double getVuforiaY() {
        return ypos;
    }

    public static double getVuforiaAngle() {
        return angle;
    }

    public static boolean isTargetVisible() {
        return targetVisible;
    }

    public double getVuforiaAngle2() {
        return vuforiaAngle;
    }

    public String getVuforiaTrackable() {
        return trackable;
    }
}