package org.firstinspires.ftc.teamcode.samples;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Reference for the SolversLib Pedro commands. Disabled until Constants can build a follower. */
@Disabled
@Autonomous
public class PedroCommands extends CommandOpMode {

    public Follower follower;

    public Pose pose = new Pose(72, 72, 90);

    public Path path;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        path = Paths.line(
                new Pose(0, 0, Math.toRadians(0)),
                new Pose(16, 28, Math.toRadians(90))
        ).linear(Math.toRadians(0), Math.toRadians(90));

        schedule(
                // Updates follower to follow path
                new RunCommand(() -> follower.update()),

                // HoldPointCommand
                new HoldPointCommand(follower, new Pose(0, 4, 0), false),
                new HoldPointCommand(follower, pose, true),

                // TurnCommand
                new TurnCommand(follower, Math.PI / 2, false),
                new TurnCommand(follower, 90, true, AngleUnit.DEGREES),

                // TurnToCommand
                new TurnToCommand(follower, Math.PI / 2),
                new TurnToCommand(follower, 90, AngleUnit.DEGREES),

                // FollowPathCommand
                new FollowPathCommand(follower, path),
                new FollowPathCommand(follower, path, true),
                new FollowPathCommand(follower, path, true, 1.0),
                new FollowPathCommand(follower, path, true, 1.0).setGlobalMaxPower(1.0)
        );
    }

    @Override
    public void run() {
        super.run();
    }
}
