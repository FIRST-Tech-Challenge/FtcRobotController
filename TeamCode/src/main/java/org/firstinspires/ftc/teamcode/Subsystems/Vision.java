package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Created by Ryan Lin on 1/04/2021.
 * modified by Andrew Chiang on 4/14/2021
 */

/**
 * https://github.com/OpenFTC/OpenCV-Repackaged
 * Open CV library
 */

public class Vision {
    private HardwareMap hardwareMap;
    private Robot robot;

    public enum Color {
        RED,
        BLUE,
    }
    private Color alliance;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    private static final String WEBCAM_NAME2 = "Webcam 2"; // insert webcam name from configuration if using webcam
    WebcamName webcamName = null;

    private static final String VUFORIA_KEY =
            "ATDGULf/////AAABmRRGSyLSbUY4lPoqBYjklpYqC4y9J7bCk42kjgYS5KtgpKL8FbpEDQTovzZG8thxB01dClvthxkSuSyCkaZi+JiD5Pu0cMVre3gDwRvwRXA7V9kpoYyMIPMVX/yBTGaW8McUaK9UeQUaFSepsTcKjX/itMtcy7nl1k84JChE4i8whbinHWDpaNwb5qcJsXlQwJhE8JE7t8NMxMm31AgzqjVf/7HwprTRfrxjTjVx5v2rp+wgLeeLTE/xk1JnL3fZMG6yyxPHgokWlIYEBZ5gBX+WJfgA+TDsdSPY/MnBp5Z7QxQsO9WJA59o/UzyEo/9BkbvYJZfknZqeoZWrJoN9jk9sivFh0wIPsH+JjZNFsPw";

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // Define constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Define where camera is in relation to center of robot in inches
    final float CAMERA_FORWARD_DISPLACEMENT  = 6.0f * mmPerInch;
    final float CAMERA_VERTICAL_DISPLACEMENT = 6.5f * mmPerInch;
    final float CAMERA_LEFT_DISPLACEMENT     = -0.75f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    OpenGLMatrix robotFromCamera = null;
    private VuforiaLocalizer vuforia = null;

    private boolean targetVisible = false;
    private VectorF targetTranslation;
    private Orientation targetRotation;

    private VuforiaTrackables targetsUltimateGoal;
    private VuforiaTrackable towerTarget;
    private VuforiaTrackable frontWallTarget;

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private int[] viewportContainerIds;

    public Vision(HardwareMap hardwareMap, Robot robot, boolean isBlue) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        if(isBlue) {
            alliance = Color.BLUE;
        } else {
            alliance = Color.RED;
        }

        webcamName = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        /**
         * This is the only thing you need to do differently when using multiple cameras.
         * Instead of obtaining the camera monitor view and directly passing that to the
         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int)}
         * on that view in order to split that view into multiple equal-sized child views,
         * and then pass those child views to the constructor.
         */
        viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);


        robot.getOpmode().telemetry.addLine("init RingPipeline");
        robot.getOpmode().telemetry.update();
        initRingPipeline();

        robot.getOpmode().telemetry.addLine("init Vuforia");
        robot.getOpmode().telemetry.update();
        initVuforia();

        robot.getOpmode().telemetry.addLine("vision init completed");
        robot.getOpmode().telemetry.update();
    }

    private void initVuforia() {
        // Configure parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[1]);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        if(alliance == Color.BLUE) {
            towerTarget = targetsUltimateGoal.get(0);
            towerTarget.setName("Blue Tower Goal Target");
        } else {
            towerTarget = targetsUltimateGoal.get(1);
            towerTarget.setName("Red Tower Goal Target");
        }
        frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // Set the position of the perimeter target as the origin (robot position will be relative to the target)
        towerTarget.setLocation(createMatrix(0, 0, mmTargetHeight, 90, 0, 0));
        frontWallTarget.setLocation(createMatrix(0, 0, mmTargetHeight, 90, 0, 0));

        OpenGLMatrix robotFromCamera = createMatrix(CAMERA_LEFT_DISPLACEMENT, CAMERA_FORWARD_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, 90, 0, 0);

        ((VuforiaTrackableDefaultListener) towerTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) frontWallTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    public boolean towerTargetScan() {
        // check the tower target to see if it is visible.
        targetsUltimateGoal.activate();
        targetVisible = false;
        if (((VuforiaTrackableDefaultListener) towerTarget.getListener()).isVisible()) {
            robot.getOpmode().telemetry.addData("Visible Target", towerTarget.getName());
            targetVisible = true;

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) towerTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in mm.
            targetTranslation = lastLocation.getTranslation();
            robot.getOpmode().telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    targetTranslation.get(0), targetTranslation.get(1), targetTranslation.get(2));

            // express the rotation of the robot in degrees.
            targetRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            robot.getOpmode().telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.1f, %.1f, %.1f", targetRotation.firstAngle, targetRotation.secondAngle, targetRotation.thirdAngle);
        }
        else {
            robot.getOpmode().telemetry.addData("Visible Target", "none");
        }
        robot.getOpmode().telemetry.update();
        return targetVisible;
    }

    public double getTowerOffsetX() {
        return targetTranslation.get(0);
    }

    public double getTowerOffsetY() {
        return targetTranslation.get(1);
    }

    public double getTowerOffsetAngle() {
        return targetRotation.thirdAngle;
    }

    private void initRingPipeline() {
        // get camera from the robot
//        int cameraMonitorViewId = this
//                .hardwareMap
//                .appContext
//                .getResources().getIdentifier(
//                        "cameraMonitorViewId",
//                        "id",
//                        hardwareMap.appContext.getPackageName()
//                );
//        if (USING_WEBCAM) {
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME2), viewportContainerIds[0]);
//        } else {
//            camera = OpenCvCameraFactory
//                    .getInstance()
//                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        }

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        pipeline = new UGContourRingPipeline(robot.getOpmode().telemetry, DEBUG);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }


    public UGContourRingPipeline.Height ringDetect() {
        return pipeline.getHeight();
    }

    // Helper method to create matrix to identify locations
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v , w));
    }
}
