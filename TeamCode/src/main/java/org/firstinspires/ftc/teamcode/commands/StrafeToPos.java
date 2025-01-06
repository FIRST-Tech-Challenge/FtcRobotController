package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class StrafeToPos extends TrajectoryCommand {
    public StrafeToPos(Vector2d pos, Drivetrain drivetrain) {
        super(null, drivetrain);
        Action action = drivetrain.getTrajectoryBuilder(drivetrain.getPose()).strafeTo(pos).build();
        trajectoryAction = action;
    }
}
