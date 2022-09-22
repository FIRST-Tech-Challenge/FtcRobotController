package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsBlinkin;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;
import org.openftc.easyopencv.OpenCvCamera;

public class StateCollectFreightWithVelocityControl extends EbotsAutonStateVelConBase{
    FreightDetector freightDetector;
    boolean freightPresent = false;
    OpenCvCamera camera;
    private static boolean exitedWithFreight = false;

    private static double stateUndoTravelDistance = 0.0;
    private final Pose startPose;

    int ballSightingScore = 0;
    int boxSightingScore = 0;
    int sightingThreshold = 2;



    public StateCollectFreightWithVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Make sure the bucket is ready for collect
        Bucket bucket = Bucket.getInstance(autonOpMode);
        if (bucket.getBucketState() != BucketState.COLLECT) {
            bucket.setState(BucketState.COLLECT);
        }

        motionController.setSpeed(Speed.SLOW);
        exitedWithFreight = false;

        startPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeadingDeg());

        // Must define

        travelDistance = 24.0;
        travelDirectionDeg = 0.0;
        targetHeadingDeg = 0.0;

        initAutonState();
        setDriveTarget();

        stateTimeLimit = 3500;

        Intake.getInstance(autonOpMode.hardwareMap).fullPower();
        freightDetector = autonOpMode.getFreightDetector();

        // turn on the lights
        EbotsBlinkin.getInstance(autonOpMode.hardwareMap).lightsOn();


//        // receive a reference to Freight detector instead of this code.
//        EbotsWebcam bucketWebCam = new EbotsWebcam(autonOpMode.hardwareMap, "bucketCam", RobotSide.FRONT, 0,-3.25f, 9.0f);
//        WebcamName webcamName = bucketWebCam.getWebcamName();
//        // With live preview
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
//        Log.d(logTag, "camera instantiated");
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                Log.d(logTag, "The camera is now open..." + stopWatchState.toString());
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                camera.setPipeline(freightDetector);
//
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                Log.d(logTag, "There was an error");
//            }
//        });
//        Log.d(logTag, "Camera for Freight Detector Instantiated");

        Log.d(logTag, "Constructor complete");

    }

    public static double getStateUndoTravelDistance() {
        return stateUndoTravelDistance;
    }

    public static boolean getExitedWithFreight(){
        return exitedWithFreight;
    }

    private void updateFreightPresent(){
        if (!freightDetector.isReadingConsumed()) {
            freightDetector.markReadingAsConsumed();
            if (freightDetector.getIsBall()) {
                Log.d(logTag, "!!! Ball Detected !!!");
                ballSightingScore++;
            } else {
                ballSightingScore--;
                ballSightingScore = Math.max(ballSightingScore, 0);
            }

            if (freightDetector.getIsBox()) {
                Log.d(logTag, "!!! Box Detected !!!");
                boxSightingScore++;
            } else {
                boxSightingScore--;
                boxSightingScore = Math.max(boxSightingScore, 0);
            }
            freightPresent = ballSightingScore >= sightingThreshold | boxSightingScore >= sightingThreshold;

        }
        telemetry.addData("Box Present", freightDetector.getIsBox());
        telemetry.addData("Ball Present", freightDetector.getIsBall());
    }

    @Override
    public boolean shouldExit() {
        // standardExitConditions include opMode inactivated, travel complete, state timed out.
        updateLocationAndError();
        updateFreightPresent();


        boolean stateTimedOut = stopWatchState.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = isTargetFieldPositionAchieved() && isTargetRotationAchieved();
        if (stateTimedOut) Log.d(logTag, "Exited because timed out. ");
        if (targetTravelCompleted) Log.d(logTag, "Target travel completed but not exiting. ");

        if(freightPresent) {
            Log.d(logTag, "Exiting because freight detected during " + this.getClass().getSimpleName());
            exitedWithFreight = true;
        }

        return freightPresent | stateTimedOut;
    }

    @Override
    public void performStateActions() {
        super.performStateActions();
    }

    @Override
    public void performTransitionalActions() {
        super.performTransitionalActions();
        EbotsBlinkin.getInstance(autonOpMode.hardwareMap).lightsOff();

        // Now pass the strafe clicks to the opmode for processing
        int avgClicksTraveled = motionController.getAverageClicks();
        autonOpMode.setForwardClicksCollect(avgClicksTraveled);
        Log.d(logTag, "Setting forwardClicksCollect to " + String.format("%d", avgClicksTraveled));

        // move the bucket to travel position
        Bucket.getInstance(autonOpMode).setState(BucketState.TRAVEL);

        Intake.getInstance(autonOpMode.hardwareMap).stop();

        stateUndoTravelDistance = new PoseError(startPose, currentPose, autonOpMode).getMagnitude();

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
