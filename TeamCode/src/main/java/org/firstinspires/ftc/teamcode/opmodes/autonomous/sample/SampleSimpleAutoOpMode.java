package org.firstinspires.ftc.teamcode.opmodes.autonomous.sample;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.SimpleAutoOpMode;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "sample simple auto mode", preselectTeleOp = "main_teleop")
public class SampleSimpleAutoOpMode extends SimpleAutoOpMode {

    @Override
    protected void setupWaypointsAndActions() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0, 0, new Rotation2d(0)));
        waypoints.add(new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
        waypoints.add(new Pose2d(2, 0, new Rotation2d(Math.PI)));
        wayPointsStack.push(waypoints);

        actions.push(new RunCommand(() -> {}));
    }
}
