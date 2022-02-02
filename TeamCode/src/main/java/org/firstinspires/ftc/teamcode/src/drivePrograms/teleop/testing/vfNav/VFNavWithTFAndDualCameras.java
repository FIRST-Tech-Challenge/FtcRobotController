package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing.vfNav;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.src.utills.VuforiaKey;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "-VFNavWithTFAndDualCameras")
public class VFNavWithTFAndDualCameras extends GenericOpModeTemplate {
    private static final String VUFORIA_KEY = VuforiaKey.VUFORIA_KEY;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    OpenGLMatrix leftCameraLocationOnRobot;
    OpenGLMatrix rightCameraLocationOnRobot;
    boolean usingLeftCamera;
    WebcamName leftCam;
    WebcamName rightCam;
    VuforiaLocalizer.Parameters parameters;
    List<VuforiaTrackable> allTrackables;
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;
    private SwitchableCamera switchableCamera;
    private boolean targetVisible = false;
    private TFObjectDetector tfod;

    private void initVFTracking() {
        telemetry.addData("Initialization", "Beginning Vuforia Initialization");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        identifyTarget(0, "Blue Storage", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall", halfTile, halfField, mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", halfTile, -halfField, mmTargetHeight, 90, 0, 180);

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
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float LEFT_CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float LEFT_CAMERA_VERTICAL_DISPLACEMENT = 7.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float LEFT_CAMERA_LEFT_DISPLACEMENT = -6.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens
        leftCameraLocationOnRobot = OpenGLMatrix
                .translation(LEFT_CAMERA_FORWARD_DISPLACEMENT, LEFT_CAMERA_LEFT_DISPLACEMENT, LEFT_CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        final float RIGHT_CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float RIGHT_CAMERA_VERTICAL_DISPLACEMENT = 7.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float RIGHT_CAMERA_LEFT_DISPLACEMENT = -6.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens
        rightCameraLocationOnRobot = OpenGLMatrix
                .translation(RIGHT_CAMERA_FORWARD_DISPLACEMENT, RIGHT_CAMERA_LEFT_DISPLACEMENT, RIGHT_CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

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

    void initTFODDetection() {
        telemetry.addData("Initialization", "Loading TFOD Context");
        telemetry.update();
        //Some pre-initialization
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;


        //Runs initialization Code
        telemetry.addData("Initialization", "Instantiating TFOD Engine");
        telemetry.update();
        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        telemetry.addData("Initialization", "Loading TFOD Assets");
        telemetry.update();
        this.tfod.loadModelFromAsset(AutoObjDetectionTemplate.TFOD_MODEL_ASSET, AutoObjDetectionTemplate.LABELS);

        telemetry.addData("Initialization", "Activating TFOD Engine");
        telemetry.update();
        tfod.activate();
        tfod.setZoom(1.4, 16.0 / 9.0);
    }

    @Override
    public void opModeMain() throws InterruptedException {

        initVFTracking();
        initTFODDetection();
        setLeftCamera();
        telemetry.addData("Initialization", "Finished");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.addData("Using Left Camera", usingLeftCamera);
            telemetry.addData("Recognition", AutoObjDetectionTemplate.findPositionOfMarker(tfod));
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targets.deactivate();
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    void switchCamera() {
        if (usingLeftCamera) { //switch to right cam
            setRightCamera();
        } else { //switch to left cam
            setLeftCamera();
        }

    }

    void setLeftCamera() {
        switchableCamera.setActiveCamera(leftCam);
        for (VuforiaTrackable trackable : allTrackables) {
            assert parameters.cameraName != null;
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, leftCameraLocationOnRobot);
        }
        usingLeftCamera = true;
    }

    void setRightCamera() {
        switchableCamera.setActiveCamera(rightCam);
        for (VuforiaTrackable trackable : allTrackables) {
            assert parameters.cameraName != null;
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, rightCameraLocationOnRobot);
        }
        usingLeftCamera = false;
    }

}
