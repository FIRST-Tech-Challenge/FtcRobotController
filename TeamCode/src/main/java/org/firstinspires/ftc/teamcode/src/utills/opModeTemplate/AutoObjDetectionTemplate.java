package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.VuforiaKey;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A template for all Autonomous opModes that use Vision, allows easy initialization
 */
@Disabled
public abstract class AutoObjDetectionTemplate extends AutonomousTemplate {

    /**
     * The file that is loaded to detect from
     */
    public static final String TFOD_MODEL_ASSET = "State Pink Team Marker Mk3.tflite";

    /**
     * The labels in the file
     */
    public static final String[] LABELS = {"Pink Team Marker v2"};

    /**
     * A object to lock on to for the thread safety, used in _initVuforia
     */
    private static final Lock VF_Lock = new ReentrantLock();

    /**
     * A object to lock on to for the thread safety, used in _initTfod
     */
    private static final Lock TFOD_Lock = new ReentrantLock();

    /**
     * vuforia object
     */
    public volatile VuforiaLocalizer vuforia;

    /**
     * Tensorflow Object Detection Object
     */
    public volatile TFObjectDetector tfod;

    protected String CameraNameToUse = GenericOpModeTemplate.LeftWebcamName;

    /**
     * Initializes the Vuforia object
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void initVuforia() throws InterruptedException {
        telemetry.addData("Vuforia initialization:", "Started");
        telemetry.update();

        checkStop();
        this._initVuforia();
        checkStop();

        telemetry.addData("Vuforia initialization:", "Complete");
        telemetry.update();

    }

    /**
     * Does the initialization, provides synchronization for thread safety
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    private void _initVuforia() throws InterruptedException {
        //does the initialization
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VuforiaKey.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, this.CameraNameToUse);

        //Waits for mutex to be available
        VF_Lock.lockInterruptibly();
        try {
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        } finally {
            VF_Lock.unlock();
        }
        checkStop();


    }

    /**
     * Initializes the Tensorflow Object Detection Object object
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void initTfod() throws InterruptedException {
        telemetry.addData("TFOD initialization:", "Started");
        telemetry.update();
        checkStop();

        this._initTfod();

        checkStop();
        telemetry.addData("TFOD initialization:", "Complete");
        telemetry.update();

    }

    /**
     * Does the initialization, provides synchronization for thread safety
     */
    private void _initTfod() throws InterruptedException {

        //Some pre-initialization
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        //Waits for mutex to be available
        TFOD_Lock.lockInterruptibly();
        try {
            //Runs initialization Code
            this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        } finally {
            TFOD_Lock.unlock();
        }

    }

    /**
     * Activates Tensorflow
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void activateTF() throws InterruptedException {
        checkStop();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.4, 16.0 / 9.0);
        }
        checkStop();
    }


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll() throws InterruptedException {
        this.initVuforia();
        this.initTfod();
        this.activateTF();
        super.initAll();
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

    public static BarcodePositions findPositionOfMarker(TFObjectDetector tfod) {
        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions == null || (recognitions.size() == 0)) {
            return BarcodePositions.NotSeen;
        } else {
            return BarcodePositions.getRecognitionLocation(recognitions.get(0));
        }
    }

    /**
     * Uses the camera to determine where the object is on screen
     *
     * @return Where the marker is
     */
    public BarcodePositions findPositionOfMarker() {
        return AutoObjDetectionTemplate.findPositionOfMarker(this.tfod);
    }

    public void dropOffFreight() throws InterruptedException {
        dropOffFreight(BarcodePositions.Right);
    }

    public void dropOffFreight(BarcodePositions Pos) throws InterruptedException {
        switch (Pos) {
            case NotSeen:
            case Right:
                // got to the top level when right
                slide.setTargetLevel(HeightLevel.TopLevel);
                break;

            case Center:
                slide.setTargetLevel(HeightLevel.MiddleLevel);
                break;

            case Left:
                // go to bottom when left
                slide.setTargetLevel(HeightLevel.BottomLevel);
                break;

        }

        //Waits for the slide to get to it's position

        double[] initialPos = {gps.getX(), gps.getY()};
        slide.setMotorPower(1);

        //Strafes forward while the distance from the wall is less than 24 in
        driveSystem.strafeAtAngle(180, 0.5);

        double currentWallDistance;
        do {
            if (MiscUtils.distance(initialPos[0], initialPos[1], gps.getX(), gps.getY()) > 24) {
                break;
            }
            currentWallDistance = Math.abs((frontDistanceSensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(gps.getRot() - 270)));
        } while (currentWallDistance < 23 && opModeIsActive() && !isStopRequested());


        driveSystem.stopAll();
        intake.setServoOpen();
        Thread.sleep(750);
        driveSystem.move(0, 5, 1, new DistanceTimeoutWarning(500));
        intake.setServoClosed();
        slide.setTargetLevel(HeightLevel.Down);
        slide.setMotorPower(0);

    }

    protected double pickUpBlock2(double distanceDriven, double startingDistanceFromWall) throws InterruptedException {

        //Loops while the item is not detected
        outer:
        while (opModeIsActive() && !isStopRequested()) {

            //Strafes forward in steps of four inches each time
            distanceDriven += 4;  //Four is currently the distance it steps each time
            driveSystem.strafeAtAngle(0, 0.3);
            double currentDistance = 0;
            double previousDistance;
            double tmpMeasure;
            double positiveIfBlue = -1;


            while (opModeIsActive() && !isStopRequested()) {

                previousDistance = currentDistance;
                currentDistance = gps.getY();

                if (frontDistanceSensor.getDistance(DistanceUnit.INCH) < (startingDistanceFromWall - distanceDriven)) {
                    driveSystem.stopAll();
                    break;
                }

                if (intakeDistanceSensor.getDistance(DistanceUnit.INCH) < 3) {
                    intake.setIntakeOff();
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(500);
                    break;
                }
                if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
                    intake.setIntakeReverse();
                    break outer;
                }

                final double minimumDistanceThreshold = .1;

                //If it detects that it is stuck
                if (Math.abs(currentDistance - previousDistance) < minimumDistanceThreshold) {
                    telemetry.addData("Warning", "Stuck");
                    telemetry.update();
                    driveSystem.stopAll();
                    driveSystem.strafeAtAngle(180, 0.3);
                    Thread.sleep(500);
                    driveSystem.stopAll();
                    gps.setPos(gps.getX(), frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6, gps.getRot());
                    tmpMeasure = frontDistanceSensor.getDistance(DistanceUnit.INCH);
                    try {
                        driveSystem.moveTowardsPosition(gps.getX() - (7 * positiveIfBlue), gps.getY() - tmpMeasure, gps.getRot() + (10 * positiveIfBlue), .3, 3, new MovementException[]{new DistanceTimeoutWarning(500), new DistanceSensorException(intakeDistanceSensor, 7.62)});
                    } catch (DistanceSensorException d) {
                        driveSystem.strafeAtAngle(180, 1);
                        Thread.sleep(200);
                        driveSystem.stopAll();
                        break;
                    } catch (MovementException ignored) {
                    }
                    continue;

                }
                Thread.sleep(40);
            }


            driveSystem.stopAll();
            ElapsedTime time = new ElapsedTime();
            while ((time.seconds() < 1.5) && (opModeIsActive() && !isStopRequested())) {
                intake.setIntakeOn();
                if ((intake.identifyContents() != FreightFrenzyGameObject.EMPTY)) {
                    break outer;
                }
            }
        }

        return distanceDriven;
    }

    protected double pickUpBlock(double distanceDriven, double startingDistanceFromWall) {
        outer:
        while (opModeIsActive() && !isStopRequested()) {
            distanceDriven += 2;  //Four is currently the distance it steps each time
            driveSystem.strafeAtAngle(0, 0.3);
            while (opModeIsActive() && !isStopRequested()) {
                if (frontDistanceSensor.getDistance(DistanceUnit.INCH) < (startingDistanceFromWall - distanceDriven)) {
                    break;
                }

                if (intakeDistanceSensor.getDistance(DistanceUnit.CM) < 6) {
                    break;
                }
                if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
                    break outer;
                }
            }
            driveSystem.stopAll();
            ElapsedTime time = new ElapsedTime();
            while ((time.seconds() < 1.5) && (opModeIsActive() && !isStopRequested())) {
                if ((intake.identifyContents() != FreightFrenzyGameObject.EMPTY)) {
                    break outer;
                }
            }
        }
        return distanceDriven;
    }

}


