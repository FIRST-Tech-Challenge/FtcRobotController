package org.firstinspires.ftc.teamcode.Extra;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Set;

public class ActionCommand implements Command {
    private final Action action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    public ActionCommand(Action action, Set<Subsystem> requirements) {
        this.action = action;
        this.requirements = requirements;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
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
        return finished;
    }
}
