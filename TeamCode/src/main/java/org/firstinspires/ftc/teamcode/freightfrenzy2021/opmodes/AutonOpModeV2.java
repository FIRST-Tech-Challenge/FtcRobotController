package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsWebcam;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.EbotsAutonState;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateCalibratingImu;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateCollectFreightWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateConfigureRoutine;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateDelayTenSeconds;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateDeliverDuck;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateEnterWarehouseForCollectVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateMoveToAllianceHubYWithOvertravelVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateOpenCVObserve;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateReverseToHubUsingVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateRotateToZeroDegreesVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToAlignTSEVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToAllowTurnToAllianceHubVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToTouchWallVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoCollectTravelWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoEnterWarehouseWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoOvertravelVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AutonOpModeV2 extends EbotsAutonOpMode {

    String logTag = "EBOTS";
    int statesCreated = 0;
    private EbotsAutonState currentState;
    private EbotsImu ebotsimu;
    private OpenCvCamera camera;
    private boolean stateComplete = false;
    private boolean allStatesCompleted = false;


    @Override
    public void runOpMode() throws InterruptedException {
        initAutonOpMode();


        Log.d(logTag, "About to start State Machine...");
        // Execute the pre-match state machine
        // Requires that the opMode is Started and the state is flagged as completed, which ensures transitional actions happen
        while (!isStarted() && !stateComplete | !isStopRequested()) {
//            Log.d(logTag, this.getClass().getSimpleName() + " : " + this.opModeIsActive());
            transitionToNextState();
            executeStateMachine();
        }

        //open the camera for freight detection
        startCamera();

        waitForStart();

        // Execute the rest of the autonStates
        while (opModeIsActive()) {
            transitionToNextState();
            executeStateMachine();
        }

        // Cleanup the resources from Vuforia
//        navigatorVuforia.deactivateTargets();

    }

    @Override
    public void initAutonOpMode() {
        telemetry.addData("Initializing AutonOpMode ", this.getClass().getSimpleName());
        telemetry.addLine("Please hold................");
        telemetry.update();

        // must initialize the following
        // touchSensors for configuration
        // imu and zeroHeadingDeg (start with assumption that starting pose is Blue Carousel)
        // currentPose
        // front web cam
        // initialize Navigator(s) (optional) and Arbitrator if more than 1
        // motion controller

        // TODO: figure out if changing these values is beneficial
        frontWebcam = new EbotsWebcam(hardwareMap, "Webcam 1", RobotSide.FRONT, 0,-3.25f, 9.0f);

        // initialize Navigator(s) (optional) and Arbitrator if more than 1
//        navigatorVuforia = new NavigatorVuforia(frontWebcam, hardwareMap);

        // motion controller
        this.motionController = new AutonDrive(this);

        // put bucket in collect position
        bucket = Bucket.getInstance(this);
        bucket.setState(BucketState.TRAVEL);

        // Setup the pre-match autonStates
        itinerary.add(StateConfigureRoutine.class);
        itinerary.add(StateCalibratingImu.class);
        itinerary.add(StateOpenCVObserve.class);

//        itinerary.add(StateCollectFreightWithVelocityControl.class);
//        itinerary.add(StateUndoCollectTravelWithVelocityControl.class);
//        itinerary.add(StateDelayTenSeconds.class);

        telemetry.addLine("Initialization complete!");
        telemetry.update();
    }

    private void executeStateMachine(){
        while (!stateComplete) {
            if (currentState.shouldExit()) {
                currentState.performTransitionalActions();
                stateComplete = true;
            } else {
                currentState.performStateActions();
                updateTelemetry();
            }
        }
//        Log.d(logTag, "....Completed state " + currentState.getClass().getSimpleName());
    }

    private void transitionToNextState(){
        // get the next state if exists
        if (itinerary.size() > 0){
            stateComplete = false;
            Class nextStateClass = itinerary.remove(0);
            currentState = EbotsAutonState.get(nextStateClass, this);
            logNewlyCreatedState(currentState);
            telemetry.clearAll();
        } else if(!allStatesCompleted){
            allStatesCompleted = true;
            Log.d(logTag, "Exiting State machine -->No more states in routine!!!");
        }
    }

    private void logNewlyCreatedState(EbotsAutonState newState){
        statesCreated++;
        String intfmt = "%d";
        String strStateCount = String.format(intfmt, statesCreated);

        try{
            Log.d(logTag, "State #" + strStateCount + " created type " + newState.getClass().getSimpleName());
            String poseString = currentPose == null ? "NULL" : currentPose.toString();
            Log.d(logTag, "Pose when state created: " + poseString);
        } catch (NullPointerException e){
            Log.d(logTag, "Error creating state #" + strStateCount + ".  Returned Null");
            Log.d(logTag, e.getStackTrace().toString());
        } catch (Exception e) {
            Log.d(logTag, "Exception encountered " + e.getStackTrace().toString());
        }
    }

    private void updateTelemetry(){
        ebotsimu = EbotsImu.getInstance(hardwareMap);
        telemetry.addData("Current State", currentState.getClass().getSimpleName());
        telemetry.addData("Current heading", ebotsimu.getCurrentFieldHeadingDeg(false));
        telemetry.update();
    }

    //Open up the camera for Freight Detection
    private void startCamera(){
        freightDetector = new FreightDetector();


        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Log.d(logTag, "cameraMonitorViewId set");
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

    }



}
