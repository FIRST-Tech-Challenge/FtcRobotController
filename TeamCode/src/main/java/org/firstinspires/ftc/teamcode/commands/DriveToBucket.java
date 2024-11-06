package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveToBucket extends CommandBase {
    private Drivetrain drivetrain;

    private Action action;
    private boolean isFinished = false;
    public DriveToBucket(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        final Vector2d basketPos = new Vector2d(-58.923881554, -55.0502525317);
        this.action = this.drivetrain.getTrajectoryBuilder(this.drivetrain.getPose())
                .strafeToLinearHeading(basketPos, Math.toRadians(45))
                .build();
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        isFinished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
