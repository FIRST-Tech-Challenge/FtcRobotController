package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.function.DoubleSupplier;

public class MoveWristCommand extends CommandBase {

    public DoubleSupplier frontward, backward;
    private WristSubsystem subsystem;

    public MoveWristCommand(WristSubsystem subsystem, DoubleSupplier frontward, DoubleSupplier backward) {
        this.subsystem = subsystem;
        this.frontward = frontward;
        this.backward = backward;
        addRequirements(subsystem);
    }

    public MoveWristCommand(WristSubsystem subsystem, DoubleSupplier directionSupplier) {
        this(subsystem, directionSupplier, () -> 0);
    }

    FTCDashboardPackets packets = new FTCDashboardPackets();

    @Override
    public void execute() {
        System.out.println("MOVE DATA: "+subsystem.isBusy()+", "+subsystem.getPosition());
        subsystem.moveWrist(Math.min(1d, Math.max(-1d, frontward.getAsDouble())), Math.min(1d, Math.max(-1d, backward.getAsDouble())));
    }
}
