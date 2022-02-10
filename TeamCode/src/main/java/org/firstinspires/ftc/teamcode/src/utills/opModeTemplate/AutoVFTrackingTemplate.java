package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.src.utills.VuforiaKey.VUFORIA_KEY;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
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

public abstract class AutoVFTrackingTemplate extends AutoObjDetectionTemplate {
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    /*
     * Create a transformation matrix describing where the camera is on the robot.
     *
     * Info:  The coordinate frame for the robot looks the same as the field.
     * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
     * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
     *
     * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
     * with the wide (horizontal) axis of the camera aligned with the X axis, and
     * the Narrow (vertical) axis of the camera aligned with the Y axis
     *
     * But, this example assumes that the camera is actually facing forward out the front of the robot.
     * So, the "default" camera position requires two rotations to get it oriented correctly.
     * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
     * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
     *
     *
     *
     */
    private static final float LEFT_CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
    private static final float LEFT_CAMERA_VERTICAL_DISPLACEMENT = 7.0f * mmPerInch;   // eg: Camera is 7 Inches above ground
    private static final float LEFT_CAMERA_LEFT_DISPLACEMENT = -6.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens
    private static final OpenGLMatrix leftCameraLocationOnRobot = OpenGLMatrix
            .translation(LEFT_CAMERA_FORWARD_DISPLACEMENT, LEFT_CAMERA_LEFT_DISPLACEMENT, LEFT_CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));


    private static final float RIGHT_CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
    private static final float RIGHT_CAMERA_VERTICAL_DISPLACEMENT = 7.0f * mmPerInch;   // eg: Camera is 7 Inches above ground
    private static final float RIGHT_CAMERA_LEFT_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens
    private static final OpenGLMatrix rightCameraLocationOnRobot = OpenGLMatrix
            .translation(RIGHT_CAMERA_FORWARD_DISPLACEMENT, RIGHT_CAMERA_LEFT_DISPLACEMENT, RIGHT_CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

    private boolean usingLeftCamera;
    private WebcamName leftCam;
    private WebcamName rightCam;
    private VuforiaLocalizer.Parameters parameters;
    private List<VuforiaTrackable> allTrackables;

    // Class Members
    private VuforiaTrackables targets = null;
    private SwitchableCamera switchableCamera;

    @Override
    public void initAll() throws InterruptedException {
        super.initAll();
        this.changeToLeftCamera();
    }

    @Override
    public void initVuforia() {
        telemetry.addData("Initialization", "Beginning Vuforia Initialization");
        telemetry.update();
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        // Indicate that we wish to be able to switch cameras.
        leftCam = hardwareMap.get(WebcamName.class, GenericOpModeTemplate.LeftWebcamName);
        rightCam = hardwareMap.get(WebcamName.class, GenericOpModeTemplate.RightWebcamName);
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(leftCam, rightCam);


        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        telemetry.addData("Initialization", "Instantiating Vuforia Engine");
        telemetry.update();
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Initialization", "Setting Up Cameras");
        telemetry.update();
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        assert switchableCamera != null;
        switchableCamera.setActiveCamera(leftCam);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        telemetry.addData("Initialization", "Loading Tracking Engine");
        telemetry.update();
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>(targets);

        /*
          In order for localization to work, we need to tell the system where each target is on the field, and
          where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
          Transformation matrices are a central, important concept in the math here involved in localization.
          See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
          for detailed information. Commonly, you'll encounter transformation matrices as instances
          of the {@link OpenGLMatrix} class.

          If you are standing in the Red Alliance Station looking towards the center of the field,
              - The X axis runs from your left to the right. (positive from the center to the right)
              - The Y axis runs from the Red Alliance Station towards the other side of the field
                where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
              - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)

          Before being transformed, each target image is conceptually located at the origin of the field's
           coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(1, "Blue Alliance Wall", 144 * mmPerInch, 58 * mmPerInch, mmTargetHeight, 90, 0, 0);
        identifyTarget(0, "Blue Storage", 106 * mmPerInch, 144 * mmPerInch, mmTargetHeight, 90, 0, 90);

        identifyTarget(2, "Red Storage", 35 * mmPerInch, 144 * mmPerInch, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", 0, 58 * mmPerInch, mmTargetHeight, 90, 0, 180);


        telemetry.addData("Initialization", "Updating Tracking Engine");
        telemetry.update();
        for (VuforiaTrackable trackable : allTrackables) {
            assert parameters.cameraName != null;
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, leftCameraLocationOnRobot);
        }
        telemetry.addData("Initialization", "Activating Vuforia Tracking Engine");
        telemetry.update();
        targets.activate();
    }

    public void changeToLeftCamera() {
        switchableCamera.setActiveCamera(leftCam);
        for (VuforiaTrackable trackable : allTrackables) {
            assert parameters.cameraName != null;
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, leftCameraLocationOnRobot);
        }
        usingLeftCamera = true;
    }

    public void changeToRightCamera() {
        switchableCamera.setActiveCamera(rightCam);
        for (VuforiaTrackable trackable : allTrackables) {
            assert parameters.cameraName != null;
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, rightCameraLocationOnRobot);
        }
        usingLeftCamera = false;
    }

    public void deactivateAll() {
        targets.deactivate();
        tfod.deactivate();
    }

    public void switchCamera() {
        if (usingLeftCamera) { //switch to right cam
            changeToRightCamera();
        } else { //switch to left cam
            changeToLeftCamera();
        }

    }

    /**
     * A getter for the location of the robot based on Vuforia Localization
     *
     * @return A array of 3 doubles in the layout of X, Y, Z. Returns null if nothing is seen
     */
    public Double[] getLocation() {
        OpenGLMatrix lastLocation = null;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (lastLocation == null) return null;
        VectorF translation = lastLocation.getTranslation();


        return new Double[]{
                (double) (translation.get(0) / mmPerInch), (double) (translation.get(1) / mmPerInch), (double) (translation.get(2) / mmPerInch)
        };
    }

    /**
     * Gets the name of what the camera sees. Returns null if nothing is seen
     *
     * @return Gets the name of what the camera sees. Returns null if nothing is seen
     */
    public String getVisibleTargetName() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return trackable.getName();
            }
        }
        return null;
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx,dy,dz  Target offsets in x,y,z axes
     * @param rx,ry,rz  Target rotations in x,y,z axes
     */
    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
