package org.firstinspires.ftc.teamcode.Intake;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.IntakeTransferHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Transfer.TransferFSM;

import java.util.concurrent.TimeUnit;

public class IntakeFSM {

    enum State {
        RAMPING_UP_TO_INTAKE,
        RAMPING_UP_TO_EJECT,
        STOPPING,
        READY_TO_INTAKE,
        EJECTING,
        STOPPED,
    }

    public RollerFSM Roller;


    private State currentState = State.RAMPING_UP_TO_INTAKE;
    private Telemetry telemetry;
    Timing.Timer autoReverseTimer;

    TransferFSM transferFSM;

    Logger logger;

    public IntakeFSM(IntakeTransferHardwareMap hardwareMap, Telemetry telemetry, TransferFSM transferFSM, Logger logger) {
        this.transferFSM = transferFSM;
        autoReverseTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        Roller = new RollerFSM(hardwareMap, telemetry, logger);
        this.telemetry = telemetry;
        this.logger = logger;
    }

    public void updateState(boolean D_Pad_Up_Press, boolean D_Pad_Left_Press) {
        Roller.updateState();

        findTargetState(D_Pad_Up_Press, D_Pad_Left_Press);
        switch (currentState) {

            case RAMPING_UP_TO_INTAKE:
                Roller.intake();
                if (Roller.INTAKING()) {
                    currentState = State.READY_TO_INTAKE;
                }
                break;

            case STOPPING:
                Roller.stop();
                if (Roller.STOPPED()) {
                    currentState = State.STOPPED;
                }

                break;

            case RAMPING_UP_TO_EJECT:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.EJECTING;
                }
                break;
        }
    }

    public void findTargetState(boolean D_Pad_Up_Press, boolean D_Pad_Left_Press) {

        if (D_Pad_Up_Press && (currentState == State.READY_TO_INTAKE || currentState == State.STOPPED || currentState == State.RAMPING_UP_TO_INTAKE)) {
            currentState = State.RAMPING_UP_TO_EJECT;

        } else if (D_Pad_Up_Press && (Roller.EJECTING())) {
            currentState = State.RAMPING_UP_TO_INTAKE;

        }

        if (D_Pad_Left_Press && (currentState == State.READY_TO_INTAKE || currentState == State.EJECTING)) {
            currentState = State.STOPPING;
        } else if (D_Pad_Left_Press && currentState == State.STOPPED) {
            currentState = State.RAMPING_UP_TO_INTAKE;
        }
    }

    public void log() {
        logger.log("Intake Current State ", currentState, Logger.LogLevels.DEBUG);
        Roller.log();
    }
}

