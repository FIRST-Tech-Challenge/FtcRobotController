package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsWebcam;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.EbotsAutonState;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateCollectFreight;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateConfigureRoutine;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateNavigateToWarehouse;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateOpenCVObserve;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateRotateToCollect;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.navigators.NavigatorVuforia;

@Autonomous
public class AutonOpMode_CollectFreight extends EbotsAutonOpMode {

    String logTag = "EBOTS";
    int statesCreated = 0;
    private EbotsAutonState currentState;
    private boolean stateComplete = false;


    @Override
    public void runOpMode() throws InterruptedException {
        initAutonOpMode();


        Log.d(logTag, "About to start State Machine...");
        // Execute the pre-match state machine
        while (!isStarted()) {
            transitionToNextState();
            executeStateMachine();
        }

        waitForStart();

        // Execute the rest of the autonStates
        while (opModeIsActive()) {
            transitionToNextState();
            executeStateMachine();
        }

        // Cleanup the resources from Vuforia
        navigatorVuforia.deactivateTargets();

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
        navigatorVuforia = new NavigatorVuforia(frontWebcam, hardwareMap);

        // motion controller
        this.motionController = new AutonDrive(this);

        // put bucket in collect position
        bucket = Bucket.getInstance(this);
        bucket.setState(BucketState.TRAVEL);

        // Setup the pre-match autonStates
        itinerary.add(StateConfigureRoutine.class);
        itinerary.add(StateOpenCVObserve.class);
        itinerary.add(StateNavigateToWarehouse.class);
        itinerary.add(StateRotateToCollect.class);
        itinerary.add(StateCollectFreight.class);
//        itinerary.add(StateObserveTeamShippingElement.class);

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
        } else {
//            Log.d(logTag, "No more states in routine!!!");
        }
    }

    private void logNewlyCreatedState(EbotsAutonState newState){
        statesCreated++;
        String intfmt = "%d";
        String strStateCount = String.format(intfmt, statesCreated);

        try{
            Log.d(logTag, "State #" + strStateCount + " created type " + newState.getClass().getSimpleName());
        } catch (NullPointerException e){
            Log.d(logTag, "Error creating state #" + strStateCount + ".  Returned Null");
            Log.d(logTag, e.getStackTrace().toString());
        } catch (Exception e) {
            Log.d(logTag, "Exception encountered " + e.getStackTrace().toString());
        }
    }

    private void updateTelemetry(){
        telemetry.addData("Current State", currentState.getClass().getSimpleName());
        telemetry.addData("Current heading", EbotsImu.getInstance(hardwareMap).getCurrentFieldHeadingDeg(false));
        telemetry.update();
    }


}
