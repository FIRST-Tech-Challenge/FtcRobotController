package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.ouropencv.ContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * A template for all Autonomous opModes that use Vision, allows easy initialization
 */
@SuppressWarnings("unused")
@Disabled
public abstract class AutoObjDetectionTemplateCV extends AutonomousTemplate {

    private static final double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    private static final double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    private static final double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    private static final double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    // Pink Range                                      Y      Cr     Cb
    private static final Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    private static final Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private static final int CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE = -1;
    private static final int CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE = -2;
    private final ContourPipeline myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY);
    private WebcamName camName;
    private OpenCvWebcam webcam;
    private boolean rightCameraOn;

    private final OpenCvCamera.AsyncCameraOpenListener defaultListener = new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            try {
                //Trying to open camera at given resolution
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            } catch (final OpenCvCameraException e) {
                //Fallback code if the camera fails to open
                //The reason this is here is because sometimes the camera will fail to open randomly
                //Even though the camera actually supports the given resolution
                final String message = e.getMessage();

                if (isValid(message)) {
                    //If we are here, we can possibly recover
                    final int[] newRes = getLargestResolution(getResolutionsFromErrorMessage(message));
                    webcam.startStreaming(newRes[0], newRes[1], OpenCvCameraRotation.UPRIGHT);
                    RobotLog.addGlobalWarningMessage("Camera failed to open because of resolution error, but recovered");
                } else {
                    throw e;
                }
            }
        }

        @Override
        public void onError(int errorCode) {

        }
    };

    @SuppressLint("NewApi")
    private static int[][] getResolutionsFromErrorMessage(final String errorMessage) {

        //Trim off fluff
        String strSizes = errorMessage.substring(errorMessage.indexOf('['), errorMessage.lastIndexOf(']') + 1);

        //Count number of valid resolutions passed back
        final long count = strSizes.chars().filter(ch -> ch == ',').count() + 1;

        final int[][] returnContainer = new int[(int) count][2];

        strSizes = strSizes.replaceAll("[,]", " ");

        int storageIndex = 0;

        while (strSizes.length() > 0) {
            //Gets the resolution without brackets, eg: 176x144
            final String resolution = strSizes.substring(strSizes.indexOf('[') + 1, strSizes.indexOf(']'));
            final String[] xy = resolution.split("[x]");
            returnContainer[storageIndex][0] = Integer.parseInt(xy[0]);
            returnContainer[storageIndex][1] = Integer.parseInt(xy[1]);

            storageIndex++;


            strSizes = strSizes.substring(strSizes.indexOf(']') + 1);


        }


        return returnContainer;
    }

    private static int[] getLargestResolution(final int[][] resolutions) {
        long maxSize = 0;
        int maxIndex = -1;
        for (int index = 0; index < resolutions.length; index++) {
            int[] x = resolutions[index];

            long size = (long) x[0] * x[1];
            if (size > maxSize) {
                maxSize = size;
                maxIndex = index;
            }
        }
        assert maxIndex >= 0;

        return resolutions[maxIndex];

    }

    private static boolean isValid(String message) {
        //null check
        if (message == null) {
            return false;
        }

        //Asserts that the message has brackets which indicate the resolution
        if (message.contains("[") && message.contains("]")) {
            return message.indexOf('[') < message.lastIndexOf(']');
        }
        return false;
    }

    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll() throws InterruptedException {
        this.initOpenCV();
        super.initAll();
    }

    public void switchWebcam() {
        if (rightCameraOn) {
            webcam.closeCameraDevice();
            webcam.closeCameraDeviceAsync(() -> {
            });

            camName = hardwareMap.get(WebcamName.class, GenericOpModeTemplate.LeftWebcamName);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            /*
              Here we use a special factory method that accepts multiple WebcamName arguments. It returns an
              {@link OpenCvSwitchableWebcam} which contains a couple extra methods over simply an {@link OpenCvCamera}.
             */
            webcam = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);

            webcam.setPipeline(myPipeline);

            // Webcam Streaming
            webcam.openCameraDeviceAsync(defaultListener);


            rightCameraOn = false;
        } else {
            webcam.closeCameraDevice();
            webcam.closeCameraDeviceAsync(() -> {
            });

            camName = hardwareMap.get(WebcamName.class, GenericOpModeTemplate.RightWebcamName);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            /*
              Here we use a special factory method that accepts multiple WebcamName arguments. It returns an
              {@link OpenCvSwitchableWebcam} which contains a couple extra methods over simply an {@link OpenCvCamera}.
             */
            webcam = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);

            webcam.setPipeline(myPipeline);

            webcam.openCameraDeviceAsync(defaultListener);


            rightCameraOn = true;
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

    public void initOpenCV() {
        /*
          NOTE: Many comments have been omitted from this sample for the
          sake of conciseness. If you're just starting out with EasyOpenCv,
          you should take a look at {@link InternalCamera1Example} or its
          webcam counterpart, {@link WebcamExample} first.
         */

        camName = hardwareMap.get(WebcamName.class, GenericOpModeTemplate.RightWebcamName);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /*
          Here we use a special factory method that accepts multiple WebcamName arguments. It returns an
          {@link OpenCvSwitchableWebcam} which contains a couple extra methods over simply an {@link OpenCvCamera}.
         */
        webcam = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);

        webcam.setPipeline(myPipeline);
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(defaultListener);

        rightCameraOn = true;

    }

    @Override
    protected void cleanup() {
        super.cleanup();
        //webcam.closeCameraDevice();
        webcam.closeCameraDeviceAsync(() -> RobotLog.dd("Webcam Image Processor", "Closed"));
    }

}


