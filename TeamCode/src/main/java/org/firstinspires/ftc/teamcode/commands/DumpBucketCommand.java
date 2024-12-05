package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

import java.util.Set;

public class DumpBucketCommand extends CommandBase {
    private final Action action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    // After X seconds, move the bucket back to the original position
    public static final double DUMP_RETRACT_TIME = 1.25d;

    BucketSubsystem bucketSubsystem;
    ElapsedTime elapsedTime;

    public DumpBucketCommand(BucketSubsystem bucketSubsystem) {
        this.bucketSubsystem = bucketSubsystem;
        this.elapsedTime = new ElapsedTime();
        this.action = null;
        this.requirements = null;
        addRequirements(bucketSubsystem);
    }

    public DumpBucketCommand(Action action, Set<Subsystem> requirements) {
        this.action = action;
        this.requirements = requirements;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.seconds() >= DUMP_RETRACT_TIME;
    }

    @Override
    public void initialize() {
        super.initialize();
        elapsedTime.reset();
        bucketSubsystem.moveToNormalPosition(false);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        bucketSubsystem.moveToNormalPosition(true);
    }

}
