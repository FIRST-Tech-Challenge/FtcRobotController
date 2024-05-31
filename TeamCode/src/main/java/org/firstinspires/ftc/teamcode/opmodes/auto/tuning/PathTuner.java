package org.firstinspires.ftc.teamcode.opmodes.auto.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutonContainer;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.PurePursuitAutonomousCommand;
import org.firstinspires.ftc.teamcode.org.rustlib.drive.FollowPathCommand;
import org.firstinspires.ftc.teamcode.org.rustlib.drive.Path;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;

@Disabled
@Autonomous(name = "Path Tuner")
public class PathTuner extends AutonContainer {
    @Override
    public Command getAutonomousCommand() {
        return new PurePursuitAutonomousCommand(new Pose2d(), new FollowPathCommand(Path.loadPath("red sample path"), drive));
    }
}
