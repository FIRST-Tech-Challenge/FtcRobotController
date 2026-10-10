package org.firstinspires.ftc.teamcode.Transfer;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.IntakeTransferHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

import java.util.concurrent.TimeUnit;

public class TransferFSM {

    public enum State {
        MOVING_DOWN,
        MOVING_UP,
        START_TO_MOVE,
        AT_DOWN,
        AT_UP,
    }

    private State currentState = State.START_TO_MOVE;
    private Telemetry telemetry;
    private GateFSM transferServoFSM;
    public static Timing.Timer autoMoveTimer;
    public Logger logger;

    public TransferFSM(IntakeTransferHardwareMap hardwareMap, Telemetry telemetry, Logger logger) {
        this.logger = logger;
        transferServoFSM = new GateFSM(hardwareMap, telemetry, logger);
        autoMoveTimer = new Timing.Timer(3, TimeUnit.SECONDS);
    }

    public void updateState(boolean Right_Bumper) {
        transferServoFSM.updateState();
        findTargetState(Right_Bumper);

        switch (currentState) {
            case MOVING_DOWN:
                transferServoFSM.MoveDown();
                if (transferServoFSM.AT_DOWN()) {
                    currentState = State.AT_DOWN;
                }
                break;

            case MOVING_UP:
                transferServoFSM.MoveUp();
                if (transferServoFSM.AT_UP()) {
                    currentState = State.AT_UP;
                }
                break;
        }
    }

    public void findTargetState(boolean Right_Bumper) {

        if (Right_Bumper && transferServoFSM.AT_UP()) {
            currentState = State.MOVING_DOWN;
        }

        if (currentState == State.AT_UP && transferServoFSM.AT_DOWN()) {
            currentState = State.AT_DOWN;
        }

        if (Right_Bumper && transferServoFSM.AT_DOWN()) {
            currentState = State.MOVING_UP;
        }

        if (autoMoveTimer.done() && transferServoFSM.AT_UP()) {
            autoMoveTimer.pause();
            currentState = State.MOVING_DOWN;
        }
    }

    public void log() {
        logger.log("Transfer Current State ", currentState, Logger.LogLevels.DEBUG);
        transferServoFSM.log();
        logger.log("Auto Transfer Move Timer", autoMoveTimer.elapsedTime(), Logger.LogLevels.DEBUG);
    }
}