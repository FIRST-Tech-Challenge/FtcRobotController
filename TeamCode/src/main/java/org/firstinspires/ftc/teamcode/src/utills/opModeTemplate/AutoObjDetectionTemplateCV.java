package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.ouropencv.ContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * A template for all Autonomous opModes that use Vision, allows easy initialization
 */
@Disabled
public abstract class AutoObjDetectionTemplateCV extends AutonomousTemplate {

    public static final double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static final double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static final double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static final double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    // Pink Range                                      Y      Cr     Cb
    public static final Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    public static final Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    protected String CameraNameToUse = GenericOpModeTemplate.LeftWebcamName;
    ContourPipeline myPipeline;
    private OpenCvCamera webcam;
    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;
    private double lowerruntime = 0;
    private double upperruntime = 0;

    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll() throws InterruptedException {
        this.initOpenCV();
        super.initAll();
    }

    private void initOpenCV() {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_Left"), cameraMonitorViewId);
        //OpenCV Pipeline

        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);


        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if (myPipeline.error) {
            telemetry.addData("Exception: ", myPipeline.debug);
        }
    }

    /**
     * It uses the camera to determine where the object is on the screen
     *
     * @param arraySize The number of samples to take
     * @param sleepTime How long to wait between each sample
     * @return It returns the average position of all the samples
     * @throws InterruptedException Throws exception if the opMode is stopped during function execution
     */
    public BarcodePositions getAverageOfMarker(int arraySize, int sleepTime) throws InterruptedException {

        BarcodePositions[] markerPositions = new BarcodePositions[arraySize];

        for (int i = 0; i < arraySize; i++) {
            markerPositions[i] = this.findPositionOfMarker();
            Thread.sleep(sleepTime);
            checkStop();
        }

        int sum = 0;
        for (int i = 0; i < arraySize; i++) {
            switch (markerPositions[i]) {
                case NotSeen:
                    break;
                case Right:
                    sum++;
                    break;
                case Left:
                    sum = sum + 2;
                    break;
                case Center:
                    sum = sum + 3;
                    break;
            }
        }

        int result = (int) Math.round(sum / (double) arraySize);


        switch (result) {
            case 1:
                return BarcodePositions.Right;
            case 2:
                return BarcodePositions.Left;
            case 3:
                return BarcodePositions.Center;
            default:
                return BarcodePositions.NotSeen;
        }
    }

    /**
     * Uses the camera to determine where the object is on screen
     *
     * @return Where the marker is
     */
    public BarcodePositions findPositionOfMarker() {
        if (myPipeline.getRectArea() > 2000) {
            if (myPipeline.getRectMidpointX() > 400) {
                return BarcodePositions.Right;
            } else if (myPipeline.getRectMidpointX() > 200) {
                return BarcodePositions.Center;
            } else {
                return BarcodePositions.Left;
            }
        }
        return BarcodePositions.NotSeen;
    }

}


