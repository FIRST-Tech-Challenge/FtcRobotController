package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsBlinkin;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsWebcam;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.DriveToEncoderTarget;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class StateCollectFreightWithEncoders implements EbotsAutonState{
    private EbotsAutonOpMode autonOpMode;
    private Telemetry telemetry;
    HardwareMap hardwareMap;


    private int targetClicks;
    private long stateTimeLimit;
    private StopWatch stopWatch;
    private DriveToEncoderTarget motionController;

    private String logTag = "EBOTS";
    private boolean firstPass = true;
    private double travelDistance = 24.0;

    private FreightDetector freightDetector;
    boolean freightPresent = false;
    //OpenCvCamera camera;



    public StateCollectFreightWithEncoders(EbotsAutonOpMode autonOpMode){
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");
        this.autonOpMode = autonOpMode;
        this.hardwareMap = autonOpMode.hardwareMap;
        this.telemetry = autonOpMode.telemetry;
        motionController = new DriveToEncoderTarget(autonOpMode);
        motionController.setSpeed(0.25);

        targetClicks = UtilFuncs.calculateTargetClicks(travelDistance);
        stateTimeLimit = 3000;
        stopWatch = new StopWatch();
        motionController.setEncoderTarget(targetClicks);

        Bucket bucket = Bucket.getInstance(autonOpMode);
        bucket.setState(BucketState.COLLECT);
        Intake.getInstance(hardwareMap).fullPower();

        //OpenCV pipeline from the opmode
        freightDetector = this.autonOpMode.getFreightDetector();

        EbotsBlinkin.getInstance(hardwareMap).lightsOn();

        //Moved to opmode issue#2
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Log.d(logTag, "cameraMonitorViewId set");
        EbotsWebcam bucketWebCam = new EbotsWebcam(hardwareMap, "bucketCam", RobotSide.FRONT, 0,-3.25f, 9.0f);
        WebcamName webcamName = bucketWebCam.getWebcamName();
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        Log.d(logTag, "camera instantiated");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Log.d(logTag, "The camera is now open...");
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(freightDetector);

            }
            @Override
            public void onError(int errorCode)
            {
                Log.d(logTag, "There was an error");
            }
        });
        Log.d(logTag, "Camera for Freight Detector Instantiated");

        Log.d(logTag, "Constructor complete");
        */


    }

    private void updateFreightPresent(){
        try {
            if (!freightDetector.isReadingConsumed()) {
                freightPresent = freightDetector.getIsBall() | freightDetector.getIsBox();
                freightDetector.markReadingAsConsumed();
                if (freightDetector.getIsBall()) Log.d(logTag, "!!! Ball Detected !!!");
                if (freightDetector.getIsBox()) Log.d(logTag, "!!! Box Detected !!!");
                telemetry.addData("Box Present", freightDetector.getIsBox());
                telemetry.addData("Ball Present", freightDetector.getIsBall());
            }
        } catch(NullPointerException e){
            telemetry.addLine("Freight Detector NULL in Collect Freight State");
        }

    }


    @Override
    public boolean shouldExit() {
        if(firstPass){
            Log.d(logTag, "Inside shouldExit...");
            firstPass = false;
        }
        updateFreightPresent();

        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = motionController.isTargetReached();
        if (stateTimedOut) Log.d(logTag, "Exited because timed out during " + this.getClass().getSimpleName());
        if(freightPresent) Log.d(logTag, "Exiting because freight detected during " + this.getClass().getSimpleName());
        if(targetTravelCompleted) Log.d(logTag, "Exiting because target travel completed during " + this.getClass().getSimpleName());

        return freightPresent | stateTimedOut | targetTravelCompleted | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        telemetry.addData("Avg Clicks", motionController.getAverageClicks());
        telemetry.addData("Position Reached", motionController.isTargetReached());
        telemetry.addData("Freight Detected", freightPresent);
        telemetry.addLine(stopWatch.toString());
    }

    @Override
    public void performTransitionalActions() {
        Log.d(logTag, "Inside transitional Actions...");
        motionController.stop();
        motionController.logAllEncoderClicks();
        Log.d(logTag, "Pose before offset: " + autonOpMode.getCurrentPose().toString());

        EbotsBlinkin.getInstance(hardwareMap).lightsOff();

        // Now pass the strafe clicks to the opmode for processing
        int avgClicksTraveled = motionController.getAverageClicks();
        int previousForwardTravel = autonOpMode.getForwardClicksCollect();
        int totalTravel = avgClicksTraveled + previousForwardTravel;
        autonOpMode.setForwardClicksCollect(totalTravel);
        Log.d(logTag, "Setting forward clicks to " + String.format("%d", totalTravel));

        // Reverse the intake for a few seconds to make sure we don't have multiple elements
        StopWatch stopWatchPurge = new StopWatch();
        long purgeTimeLimit = 750;
        Intake intake = Intake.getInstance(hardwareMap);
        intake.purge();
        while (autonOpMode.opModeIsActive() && stopWatchPurge.getElapsedTimeMillis() < purgeTimeLimit){
            // Just wait for the purge
        }
        intake.stop();

        // Update the robots pose in autonOpMode
        double currentHeadingRad = Math.toRadians(EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true));
        double xTravelDelta = travelDistance * Math.cos(currentHeadingRad);
        double yTravelDelta = travelDistance * Math.sin(currentHeadingRad);
        FieldPosition deltaFieldPosition = new FieldPosition(xTravelDelta, yTravelDelta);
        FieldPosition startingFieldPosition = autonOpMode.getCurrentPose().getFieldPosition();
        startingFieldPosition.offsetInPlace(deltaFieldPosition);
        Log.d(logTag, "Pose after offset: " + autonOpMode.getCurrentPose().toString());

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
