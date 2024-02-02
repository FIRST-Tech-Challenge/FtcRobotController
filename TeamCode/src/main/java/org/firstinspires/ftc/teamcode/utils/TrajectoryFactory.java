package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.constraint.DifferentialDriveVoltageConstraint;
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;

public class TrajectoryFactory {
    private static TrajectoryConfig config =
            new TrajectoryConfig(Constants.ChassisConstants.RobotMaxVelFront,
                    Constants.ChassisConstants.RobotMaxAccFront)
                    .setKinematics(Constants.ChassisConstants.kinematics);

    public static final Trajectory t1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0,Rotation2d.fromDegrees(0)),
            new ArrayList<>(),
            new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
            config);


}



