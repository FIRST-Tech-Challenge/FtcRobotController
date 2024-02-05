package org.firstinspires.ftc.teamcode.Extra;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.Subsystem;
//
//import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
//
//import java.util.Set;
//
//public class ActionCommand implements Command {
//    private TrajectorySequence trajectorySequence;
//    private final Set<Subsystem> requirements;
//    private boolean finished = false;
//
//    public ActionCommand(TrajectorySequence trajectorySequence, Set<Subsystem> requirements) {
//        this.trajectorySequence = trajectorySequence;
//        this.requirements = requirements;
//    }
//
//    @Override
//    public Set<Subsystem> getRequirements() {
//        return requirements;
//    }
//
//    @Override
//    public void execute() {
//        TelemetryPacket packet = new TelemetryPacket();
//        finished = !trajectorySequence.run(packet);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return finished;
//    }
//}
