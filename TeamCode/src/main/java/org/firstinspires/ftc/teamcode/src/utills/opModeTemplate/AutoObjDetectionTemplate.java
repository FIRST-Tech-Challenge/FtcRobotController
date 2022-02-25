package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
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

    public static BarcodePositions findPositionOfMarker(TFObjectDetector tfod) {
        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions == null || (recognitions.size() == 0)) {
            return BarcodePositions.NotSeen;
        } else if (recognitions.size() == 2) { // Handles edge case where the object is phantom duplicated
            if (BarcodePositions.getRecognitionLocation(recognitions.get(0)) == BarcodePositions.Center) {
                return BarcodePositions.getRecognitionLocation(recognitions.get(1));
            }
            return BarcodePositions.getRecognitionLocation(recognitions.get(0));
        } else {
            return BarcodePositions.getRecognitionLocation(recognitions.get(0));
        }
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

    /**
     * Activates Tensorflow
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void activateTF() throws InterruptedException {
        checkStop();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 16.0 / 9.0);
        }
        checkStop();
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

    public void dropOffFreight(BarcodePositions Pos, double tuningFactor) throws InterruptedException {
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

        ElapsedTime slideStuckTimer = new ElapsedTime();
        while (!slide.isAtPosition()) {
            if (slideStuckTimer.seconds() > 8) {
                RobotLog.addGlobalWarningMessage("Slide Was Stuck. Dropping on lowest level");
                slide.setTargetLevel(HeightLevel.BottomLevel);
                break;
            }
            Thread.sleep(40);
        }

        //Strafes forward while the distance from the wall is less than 24 in
        driveSystem.strafeAtAngle(180, 0.5);

        double currentWallDistance;
        do {
            if (MiscUtils.distance(initialPos[0], initialPos[1], gps.getX(), gps.getY()) > 24) {
                break;
            }
            currentWallDistance = Math.abs((frontDistanceSensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(gps.getRot() - 270)));
        } while (currentWallDistance < (23 + tuningFactor) && opModeIsActive() && !isStopRequested());


        driveSystem.stopAll();
        intake.setServoOpen();
        Thread.sleep(750);
        driveSystem.move(0, 5, 1, new DistanceTimeoutWarning(500));
        intake.setServoClosed();
        slide.setTargetLevel(HeightLevel.Down);
        slide.setMotorPower(0);

    }

    public void dropOffFreight(BarcodePositions pos) throws InterruptedException {
        dropOffFreight(pos, 0);
    }

    protected double pickUpBlock2(double distanceDriven, double startingDistanceFromWall, boolean isBlue) throws InterruptedException {

        //Loops while the item is not detected
        outer:
        while (opModeIsActive() && !isStopRequested()) {

            //Strafes forward in steps of four inches each time
            distanceDriven += 4;  //Four is currently the distance it steps each time
            driveSystem.strafeAtAngle(0, 0.3);
            double currentDistance = 0;
            double previousDistance;
            double turnIncrement = 0; //this is the additional amount the robot should turn to pick up each time
            double positiveIfBlue = 1;
            if (!isBlue) {
                positiveIfBlue = -1;
            }


            while (opModeIsActive() && !isStopRequested()) {

                driveSystem.strafeAtAngleWhileTurn(0, gps.getRot() + turnIncrement, .3);
                previousDistance = currentDistance;
                currentDistance = gps.getY();

                //checks if increment of 4 inches is fulfilled for this loop
                if (gps.getY() < (startingDistanceFromWall - distanceDriven)) {
                    driveSystem.stopAll();
                    break;
                }

                // checks if intake sensor is triggered
                if (intakeDistanceSensor.getDistance(DistanceUnit.INCH) < 3) {
                    intake.setIntakeOff();
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(500);
                    break;
                }

                //checks if bucket is filled
                if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
                    intake.setIntakeReverse();
                    break outer;
                }

                final double minimumDistanceThreshold = .1;

                //If it detects that it is stuck
                if (Math.abs(currentDistance - previousDistance) < minimumDistanceThreshold) {

                    // output to telemetry
                    telemetry.addData("Warning", "Stuck");
                    telemetry.update();
                    driveSystem.stopAll();

                    // back away from block
                    driveSystem.strafeAtAngle(180, .5);
                    Thread.sleep(500);
                    driveSystem.stopAll();


                    // turn 20 degrees away from wall
                    //sets the strafeAtAngle of this movement to these variable changes
                    turnIncrement += (10 * positiveIfBlue);


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

    protected double pickUpBlock(double distanceDriven, double startingDistanceFromWall, boolean isBlue) throws InterruptedException {
        boolean strafeIntoWall = false;
        double positiveIfBlue = 1;
        if (!isBlue) {
            positiveIfBlue = -1;
        }

        outer:
        while (opModeIsActive() && !isStopRequested()) {
            //Strafes forward in steps of four inches each time
            distanceDriven += 4;  //Four is currently the distance it steps each time
            driveSystem.strafeAtAngle(0, 0.3);

            double currentDistance = 0;
            double previousDistance;
            double turnIncrement = 0; //this is the additional amount the robot should turn to pick up each time

            final double minimumDistanceThreshold = .05;


            while (opModeIsActive() && !isStopRequested()) {

                if (turnIncrement > 21) {
                    turnIncrement = 0;
                }
                driveSystem.strafeAtAngleWhileTurn(0, gps.getRot() + turnIncrement, .3);
                previousDistance = currentDistance;
                currentDistance = gps.getY();

                if (frontDistanceSensor.getDistance(DistanceUnit.INCH) < (startingDistanceFromWall - distanceDriven)) {
                    break;
                }

                if (intakeDistanceSensor.getDistance(DistanceUnit.INCH) < 3) {
                    break;
                }
                if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
                    break outer;
                }

                //If it detects that it is stuck
                if (Math.abs(currentDistance - previousDistance) < minimumDistanceThreshold) {

                    // output to telemetry
                    telemetry.addData("Warning", "Stuck");
                    telemetry.update();
                    driveSystem.stopAll();

                    // back away from block
                    driveSystem.strafeAtAngle(180, .35);
                    Thread.sleep(500);
                    driveSystem.stopAll();
                    Thread.sleep(50);

                    // turn 20 degrees away from wall
                    //sets the strafeAtAngle of this movement to these variable changes
                    turnIncrement += (7 * positiveIfBlue);
                    strafeIntoWall = true;

                }

                if (Math.abs(180 - gps.getRot()) > 45) {
                    driveSystem.turnTo(180, 0.3);
                    turnIncrement = 0;
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
        if (strafeIntoWall) {

            driveSystem.moveToPosition(gps.getX() - (10 * (-positiveIfBlue)), 30, 180, 2, new DistanceTimeoutWarning(100));
        }
        return distanceDriven;
    }

}


