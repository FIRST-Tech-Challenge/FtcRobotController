package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous;

import java.util.ArrayList;
import java.util.List;

public abstract class BaseStateMachine extends BaseAutonomous {
    // List of all states the robot could be in
    public enum State {
        STATE_INITIAL,
        STATE_COMPLETE,
        LOGGING,
    }

    private final static String TAG = "BaseStateMachine";// Logging tag
    private State mCurrentState;                         // Current State Machine State.
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    /** Initializes State Machine
     * @param team current team
     */
    public void init(Team team) {
        super.init(team);
        this.msStuckDetectInit = 15000;
        this.msStuckDetectInitLoop = 15000;
        // Starts state machine
        newState(State.STATE_INITIAL);
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
            case LOGGING:
                telemetry.update();
                break;
            case STATE_INITIAL:
                // Initialize
                // Change to next state
                newState(State.STATE_COMPLETE);
                break;
            // Ending state
            case STATE_COMPLETE:
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