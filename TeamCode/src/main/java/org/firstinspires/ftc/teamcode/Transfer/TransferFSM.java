package org.firstinspires.ftc.teamcode.Transfer;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;

import java.util.concurrent.TimeUnit;

public class TransferFSM {

    public enum State {
        MOVING_DOWN,
        MOVING_UP,
        START_TO_MOVE,
        INTAKING,
        STOPPING,
        AT_DOWN,
        AT_UP,
        START_TO_INTAKE,
        STOPPED,
    }

    private State currentState = State.START_TO_MOVE;
    private Telemetry telemetry;
    private GateFSM transferServoFSM;
    private TransferMotorFSM Belt;
    public static Timing.Timer autoMoveTimer;
 /*   public Logger logger;*/

    public TransferFSM(NewIntakeTransferHWmap hardwareMap, Telemetry telemetry) {
        Belt = new TransferMotorFSM(hardwareMap, telemetry);
/*        this.logger = logger;*/
        transferServoFSM = new GateFSM(hardwareMap, telemetry);
        /*autoMoveTimer = new Timing.Timer(3, TimeUnit.SECONDS);*/
    }

    public void updateState(boolean Right_Bumper, boolean D_Pad_Left_Press) {
        Belt.updateState();
        transferServoFSM.updateState();
        findTargetState(Right_Bumper, D_Pad_Left_Press);

        switch (currentState) {

            case START_TO_INTAKE:
                Belt.Move();
                if (Belt.INTAKING()) {
                    currentState = State.INTAKING;
                }
                break;

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

            case STOPPING:
                Belt.Stop();
                if (Belt.STOPPED()) {
                    currentState = State.STOPPED;
                }
                break;
        }
    }

    public void findTargetState(boolean Right_Bumper, boolean D_Pad_Left_Press) {

        if (D_Pad_Left_Press && Belt.INTAKING()) {
            currentState = State.STOPPING;
        }
        if (D_Pad_Left_Press && Belt.STOPPED()) {
            currentState = State.START_TO_MOVE;
        }
        if (Right_Bumper && transferServoFSM.AT_UP()) {
            currentState = State.MOVING_DOWN;

        }
        if (Right_Bumper && transferServoFSM.AT_DOWN()) {
            currentState = State.MOVING_UP;
        }

/*        if (autoMoveTimer.done() && transferServoFSM.AT_UP()) {
            autoMoveTimer.pause();
            currentState = State.MOVING_DOWN;
        }*/
    }

/*    public void log() {
        logger.log("Transfer Current State ", currentState, Logger.LogLevels.DEBUG);
        transferServoFSM.log();
        logger.log("Auto Transfer Move Timer", autoMoveTimer.elapsedTime(), Logger.LogLevels.DEBUG);
    }*/
}

/*
package org.firstinspires.ftc.teamcode.intaketransfer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intaketransfer.BeltFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferServoFSM;

public class TransferFSM {


    public enum State {
        MOVING_DOWN,
        MOVING_UP,
        START_TO_MOVE,
        STOPPING,
        AT_DOWN,
        AT_UP,
        MOVING,
        STOPPED,
    }

    private State currentState = State.START_TO_MOVE;
    private BeltFSM Belt;
    private Telemetry telemetry;
    private TransferServoFSM transferServoFSM;


    public TransferFSM(Intaketransferhwmap hardwareMap, Telemetry telemetry) {
        Belt = new BeltFSM(hardwareMap, telemetry);
        this.telemetry = telemetry;
        transferServoFSM = new TransferServoFSM(hardwareMap, telemetry);
    }

    public void updateState(boolean D_Pad_Right_Press, boolean Right_Bumper) {
        Belt.updateState();
        transferServoFSM.updateState();
        findTargetState(D_Pad_Right_Press, Right_Bumper);

        switch (currentState) {

            case START_TO_MOVE:
                Belt.Move();
                if (Belt.MOVING()) {
                    currentState = State.MOVING;
                }
                break;

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

            case STOPPING:
                Belt.Stop();
                if (Belt.STOPPED()) {
                    currentState = State.STOPPED;
                }
                break;
        }
        telemetry.addData("Transfer Current State ", currentState);
    }

    public void findTargetState(boolean D_Pad_Right_Press, boolean Right_Bumper) {
        if (D_Pad_Right_Press && Belt.MOVING()) {
            currentState = State.STOPPING;
        }
        if (D_Pad_Right_Press && Belt.STOPPED()) {
            currentState = State.START_TO_MOVE;
        }
        if (Right_Bumper && transferServoFSM.AT_UP()) {
            currentState = State.MOVING_DOWN;

        }
        if (Right_Bumper && transferServoFSM.AT_DOWN()) {
            currentState = State.MOVING_UP;
        }
    }
}*/
