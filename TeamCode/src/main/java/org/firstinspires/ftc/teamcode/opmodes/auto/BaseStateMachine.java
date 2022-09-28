package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous;

import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "BaseStateMachine", group = "Autonomous")
public class BaseStateMachine extends BaseAutonomous {
    // List of all states the robot could be in
    public String teamAsset;
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    public enum State {
        IDENTIFY_TARGET,
        DRIVE_TO_MEDIUM_JUNCTION,
        POSITION_ROBOT_AT_JUNCTION,
        PLACE_CONE,
        PARK,
        LOGGING,
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
                if(teamAsset == null){
                    //drive forward slowly/10 inches and identify again
                    if(driveSystem.driveToPosition(254, DriveSystem.Direction.FORWARD, 0.5)){
                        teamAsset = "David";
                    }
                    identifySleeve();

                } else{
                    newState(State.DRIVE_TO_MEDIUM_JUNCTION);
                    break;
                }
                break;
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

}