package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Config.VisionConfig;
import org.firstinspires.ftc.teamcode.Subsystems.MinorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.DetectMarkerPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.DetectMarkerThread;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.MarkerLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * The Vision Subsystem
 *
 * @see DetectMarkerPipeline
 * @see <a href="https://github.com/OpenFTC/EasyOpenCV">EasyOpenCV</a>
 */

public class Vision extends MinorSubsystem {
    MarkerLocation finalMarkerLocation; // Marker Location

    WebcamName webcamName = null;

    // Since ImageTarget trackable use mm to specify their dimensions, we must use mm for all the physical dimension.
    // Define constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Define where camera is in relation to center of robot in inches
    final float CAMERA_FORWARD_DISPLACEMENT  = 6.0f * mmPerInch; // TODO: CALIBRATE WHEN ROBOT IS BUILT
    final float CAMERA_VERTICAL_DISPLACEMENT = 6.5f * mmPerInch;
    final float CAMERA_LEFT_DISPLACEMENT     = -0.75f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation;
    OpenGLMatrix robotFromCamera = null;
    private VuforiaLocalizer vuforia;

    private boolean targetVisible;
    private VectorF targetTranslation;
    private Orientation targetRotation;

    private OpenCvCamera camera;

    private int[] viewportContainerIds;

    /**
     * Class instantiation
     *
     * @param robot the robot
     * @throws InterruptedException It might happen because the thread is interrupted.
     */
    public Vision(Robot robot) throws InterruptedException {
        super(robot);

        webcamName = hardwareMap.get(WebcamName.class, VisionConfig.WEBCAM_NAME);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);


        telemetry.telemetry(3, "Vision:", "init Vuforia started");
        initVuforia();
        telemetry.telemetry(2, "Vision:", "init Vuforia completed");

        OpenCvInternalCamera robotCamera;

        robotCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        telemetry.telemetry(3, "Vision:", "Detecting Marker");
        DetectMarkerThread detectMarkerRunnable = new DetectMarkerThread(robot, robotCamera);
        Thread detectMarkerThread = new Thread(detectMarkerRunnable);
        detectMarkerThread.start();
        detectMarkerThread.join();
        finalMarkerLocation = detectMarkerRunnable.getMarkerLocation();
        telemetry.telemetry(2, "Vision:", "Detected Marker");
    }

    private void initVuforia() {
        // Configure parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[1]);
        parameters.vuforiaLicenseKey = VisionConfig.VUFORIA_KEY; //moved it to VisionConfig for easier access
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        OpenGLMatrix robotFromCamera = createMatrix(CAMERA_LEFT_DISPLACEMENT, CAMERA_FORWARD_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, 90, 0, 0);
    }


    // Helper method to create matrix to identify locations
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v , w));
    }
}
