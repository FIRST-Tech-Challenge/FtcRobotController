package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class StrafeWithHeading extends TrajectoryCommand {
    public StrafeWithHeading(Vector2d pos, double heading,Drivetrain drivetrain) {
        super(null, drivetrain);
        Action action = drivetrain.getTrajectoryBuilder(drivetrain.getPose()).strafeToLinearHeading(pos, heading).build();
        trajectoryAction = action;
    }
}
