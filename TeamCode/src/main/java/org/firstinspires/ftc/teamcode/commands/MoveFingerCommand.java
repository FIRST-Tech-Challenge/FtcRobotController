package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;

import java.util.function.DoubleSupplier;

public class MoveFingerCommand extends CommandBase {

    private FingerSubsystem fingerSubsystem;
    private DoubleSupplier forward, backward;

    public MoveFingerCommand(FingerSubsystem fingerSubsystem, DoubleSupplier forward, DoubleSupplier backward) {
        this.fingerSubsystem = fingerSubsystem;
        this.forward = forward;
        this.backward = backward;
    }

    @Override
    public void execute() {
        fingerSubsystem.locomoteFinger(forward.getAsDouble(), backward.getAsDouble());
    }
}
