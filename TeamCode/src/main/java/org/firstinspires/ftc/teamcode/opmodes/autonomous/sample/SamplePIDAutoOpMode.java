package org.firstinspires.ftc.teamcode.opmodes.autonomous.sample;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.PIDControlAutoOpMode;

import java.util.ArrayList;
import java.util.List;

//@Autonomous(name = "sample PID auto mode", preselectTeleOp = "main_teleop")
public class SamplePIDAutoOpMode extends PIDControlAutoOpMode {

    @Override
    protected void setupWaypointsAndActions() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0, 0, new Rotation2d(0)));
        waypoints.add(new Pose2d(1, 0, new Rotation2d(0)));
        waypoints.add(new Pose2d(2,0,new Rotation2d(0)));
//        waypoints.add(new Pose2d(2,.3,new Rotation2d(0)));
        wayPointsStack.push(waypoints);

        actions.push(new RunCommand(() -> {}));
    }
}