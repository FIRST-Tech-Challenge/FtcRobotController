package org.firstinspires.ftc.teamcode.Transfer;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;


@Config
public class TransferMotorFSM {

    public enum State {
        STOPPED,
        INTAKING,
        EJECTING
    }

    private Telemetry telemetry;
    private MotorWrapper transferMotor;
    public double currentVelocity;

    public static double targetVelocity;

    public static double power = 0.8;
    public static double movePower = 1;
    public static double stopPower = 0;

    public State State;

    public TransferMotorFSM(NewIntakeTransferHWmap hwMap, Telemetry telemetry) {
        transferMotor = new MotorWrapper(hwMap.getTransferMotor(), false, 1, false);
        this.telemetry = telemetry;
        State = State.INTAKING;
    }

    public void updateState() {
        updatePID();
        transferMotor.readVelocity();

        if (currentVelocity == 0) {
            State = State.STOPPED;
        }

        if (currentVelocity < 0) {
            State = State.EJECTING;
        }

        if (currentVelocity < 0) {
            State = State.INTAKING;
        }
        telemetry.addData("Current State ", State);
        telemetry.addData("Power ", power);
        telemetry.addData("Velocity ", transferMotor.getVelocity());

    }

    public void updatePID() {
        currentVelocity = transferMotor.getVelocity();
        transferMotor.setRPM(targetVelocity);
    }

    public void Move() {
        power = movePower;
    }

    public void Stop() {
        power = stopPower;
    }

    public boolean INTAKING() {
        return State == State.INTAKING;
    }

    public boolean EJECTING() {
        return State == State.EJECTING;
    }

    public boolean STOPPED() {
        return State == State.STOPPED;
    }
}