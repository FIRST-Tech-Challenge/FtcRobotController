package org.firstinspires.ftc.teamcode.Transfer;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.IntakeTransferHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.ServoWrapper;

import java.util.concurrent.TimeUnit;

@Config
public class GateFSM {

    public enum State {
        MOVING_TO_POSITION,
        AT_UP,
        AT_DOWN,
    }

    private Telemetry telemetry;
    private ServoWrapper transferServo;
    public State currentState;
    public static double targetPosition = 0;
    public static double positionUp = .38;
    public static double positionDown = 0;
    static Timing.Timer transferPostitionTimer;
    public Logger logger;

    public GateFSM(IntakeTransferHardwareMap intaketransferhwmap, Telemetry telemetry, Logger logger) {
        transferServo = new ServoWrapper(intaketransferhwmap.getTransferServo());
        transferPostitionTimer = new Timing.Timer(1, TimeUnit.SECONDS);
        this.logger = logger;
        this.telemetry = telemetry;
        currentState = State.AT_DOWN;
        transferServo.setPosition(.6);
    }

    public void updateState() {

        double percentError = Math.abs((transferServo.getPosition() - targetPosition) / targetPosition);
        if (!transferPostitionTimer.isTimerOn() && percentError >= 0.001) {
            currentState = State.MOVING_TO_POSITION;
            transferServo.setPosition(targetPosition);
            transferPostitionTimer.start();
        }

        if (transferPostitionTimer.done()) {
            transferPostitionTimer.pause();
            if (currentState == State.AT_UP) {
                targetPosition = positionDown;
            }
        }


        if (targetPosition == positionUp) {
            currentState = State.AT_UP;
        }

        if (targetPosition == positionDown) {
            currentState = State.AT_DOWN;
        }
    }

    public boolean AT_DOWN() {
        return currentState == State.AT_DOWN;
    }

    public boolean AT_UP() {
        return currentState == State.AT_UP;
    }

    public void MoveUp() {
        targetPosition = positionUp;
    }

    public void MoveDown() {
        targetPosition = positionDown;
    }

    public void log() {
        logger.log("Target Position ", targetPosition, Logger.LogLevels.DEBUG);
        logger.log("ServoFSM State ", currentState, Logger.LogLevels.DEBUG);
    }
}