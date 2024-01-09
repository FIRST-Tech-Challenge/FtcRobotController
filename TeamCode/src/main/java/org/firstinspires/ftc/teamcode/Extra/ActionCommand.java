package org.firstinspires.ftc.teamcode.Extra;//package org.firstinspires.ftc.teamcode.Extra;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//public class ActionCommand extends CommandBase {
//    private final Action action;
//    private TelemetryPacket packet;
//    private Boolean isFinished = false;
//    private final MecanumDrive mecanumDrive;
//
//    public ActionCommand(Action action, TelemetryPacket packet, MecanumDrive mecanumDrive){
//        this.action = action;
//        this.packet = packet;
//        this.mecanumDrive = mecanumDrive;
//    }
//
//    @Override
//    public void execute() {
//        isFinished = ! action.run(packet);
//    }
//
//
//    @Override
//    public void end(boolean interrupted) {
//        if (interrupted) {
//            mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return isFinished;
//    }
//}
