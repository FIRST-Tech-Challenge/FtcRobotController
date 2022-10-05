package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous;

import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "BaseStateMachine", group = "Autonomous")
public class BaseStateMachine extends BaseAutonomous {
    // List of all states the robot could be in
    private String teamAsset;
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private int parkStep = 0;

    public enum State {
        IDENTIFY_TARGET,
        DRIVE_TO_MEDIUM_JUNCTION,
        POSITION_ROBOT_AT_JUNCTION,
        PLACE_CONE,
        PARK,
        END_STATE,
        LOGGING,
    }

    public enum teamAsset {
        BRIAN,
        DAVID,
        TEAM,
    }

    private final static String TAG = "BaseStateMachine";// Logging tag
    private State mCurrentState;                         // Current State Machine State.
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    /** Initializes State Machine
     */
    public void init() {
        super.init();
        this.msStuckDetectInit     = 15000;
        this.msStuckDetectInitLoop = 15000;
        // Starts state machine
        vuforia = new Vuforia(hardwareMap, Vuforia.CameraChoice.WEBCAM1);
        newState(State.IDENTIFY_TARGET);


    }

    @Override
    public void init_loop() {
        if(vuforia == null){
            return;
        }
        telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());
        telemetry.update();

        identifySleeve();
    }

    private void identifySleeve() {
        if(vuforia.isTeamAssetVisible()){
            teamAsset = vuforia.identifyTeamAsset();
        }
    }

    /**
     * State machine loop
     */
    @Override
    public void loop() {
        // Update telemetry each time through loop
        telemetry.addData("State", mCurrentState);
        telemetry.update();
        // Execute state machine
        switch (mCurrentState) {
            case IDENTIFY_TARGET:
                if (teamAsset == null) {
                    //drive forward slowly/10 inches and identify again
                    //backwards is forwards

                    if (driveSystem.driveToPosition(254, DriveSystem.Direction.BACKWARD, 0.3)) {
                        telemetry.addData("on heading? ", driveSystem.onHeading(0.3, 0));
                        identifySleeve();
                        teamAsset = "Brain";
                    }
                    identifySleeve();
                    Log.d("signal sleeve", vuforia.identifyTeamAsset());
                    telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());

                } else {
                    newState(State.PARK);
                    break;
                }
                break;
            case DRIVE_TO_MEDIUM_JUNCTION:
                if (driveSystem.driveToPosition(300, DriveSystem.Direction.BACKWARD, 0.3)) {
                    newState(State.POSITION_ROBOT_AT_JUNCTION);
                }
            case PARK:
                if (teamAsset.equals("David")) {
                    if(parkState()){
                        newState(State.END_STATE);
                    }
                }
                if (teamAsset.equals("Brain")) {
                    if (driveSystem.driveToPosition(500, DriveSystem.Direction.BACKWARD, 0.5)) {
                        newState(State.END_STATE);
                    }
                }
                if (teamAsset.equals("7330")) {
                    if(parkState()){
                        newState(State.END_STATE);
                    }
                }
            case END_STATE:
                Log.d("parked", vuforia.identifyTeamAsset());
                //"david" left two squares, "brain" center two, "7330" right two squares

        }

    }

    /** Changes state to given state
     * @param newState state to change to
     */
    private void newState(State newState) {
        // Restarts the state clock as well as the state
        mStateTime.reset();
        mCurrentState = newState;
    }

    private boolean parkState() {
        if (parkStep == 0) {
            if (driveSystem.driveToPosition(400, DriveSystem.Direction.BACKWARD, 0.5)) {
                parkStep++;
            }
        }
        if (parkStep == 1) {
            if (teamAsset.equals("David")) {
                if (driveSystem.driveToPosition(600, DriveSystem.Direction.RIGHT, 0.5)) {
                    return true;
                }
            }
            if (teamAsset.equals("7330")) {
                if (driveSystem.driveToPosition(400, DriveSystem.Direction.LEFT, 0.5)) {
                    return true;
                }
            }
        }
        return false;
    }
}