package org.firstinspires.ftc.teamcode.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;


@Config
public class RollerFSM {

    public static State State;
    private final MotorWrapper intakeMotor;
    public static double kS = 0, kV = 1.5, kA = 0;
    public static double p = 0.15, i = 0, d = 0;
    private double currentVelocity;
    public static double targetVelocity = 2790;
    public static double stoppingTargetVelocity = 0;
    public static double intakingTargetVelocity = 2790;
    public static double ejectingTargetVelocity = -1400;

    private Logger logger;

    public enum State {
        STOPPED,
        INTAKING,
        EJECTING,
    }

    public RollerFSM(NewIntakeTransferHWmap hwMap, Telemetry telemetry) {
        intakeMotor = new MotorWrapper(hwMap.getIntakeMotor(), true, 1, true);
        State = State.INTAKING;
        this.logger = logger;
    }

    public void updateState() {

        if (currentVelocity == 0) {
            State = State.STOPPED;
        }

        if (currentVelocity > 50) {
            State = State.INTAKING;
        }

        if (currentVelocity < 0) {
            State = State.EJECTING;
        }

        intakeMotor.readVelocity();
        intakeMotor.setVelocityConstants(p, i, d, kS, kV, kA);
        updatePID();
    }

    public void updatePID() {
        currentVelocity = intakeMotor.getVelocity();
        intakeMotor.setVelocity(targetVelocity);
    }

    public boolean STOPPED() {
        return State == State.STOPPED;
    }

    public boolean INTAKING() {
        return State == State.INTAKING;
    }

    public boolean EJECTING() {
        return State == State.EJECTING;
    }

    public void stop() {
        targetVelocity = stoppingTargetVelocity;
    }

    public void intake() {
        targetVelocity = intakingTargetVelocity;
    }

    public void eject() {
        targetVelocity = ejectingTargetVelocity;
    }

    public void log() {
        logger.log("Roller FSM State ", State, Logger.LogLevels.DEBUG);
        logger.log("Current Velocity ", currentVelocity, Logger.LogLevels.DEBUG);
        logger.log("Target Velocity ", targetVelocity, Logger.LogLevels.DEBUG);
        logger.log("Intake motor current ", intakeMotor.getCurrent(), Logger.LogLevels.DEBUG);
    }
}