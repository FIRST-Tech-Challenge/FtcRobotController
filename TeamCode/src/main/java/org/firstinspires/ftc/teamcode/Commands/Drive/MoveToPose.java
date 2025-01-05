package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.RobotContainer;
import java.util.ArrayList;


// This command moves robot in a straight line path from
// current pose to the supplied destination pose following the
// supplied max speed (m/s) and max acceleration (m/s2)
public class MoveToPose extends CommandBase {

    // the follow-path command that this command will use
    FollowPath cmd;

    // local copy of input parameters
    double maxSpeed, maxAccel;
    Pose2d dest;

    // constructor
    public MoveToPose(double maxSpeed,
                      double maxAccel,
                      Pose2d destination) {

        // record input parameters
        this.maxSpeed= maxSpeed;
        this.maxAccel = maxAccel;
        this.dest = destination;
        addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // current robot pose
        Pose2d currentPose = RobotContainer.odometry.getCurrentPos();

        // straight-line travel heading
        Rotation2d heading = new Rotation2d(dest.getX()-currentPose.getX(),
                dest.getY()- currentPose.getY());

        // construct follow-path command
        cmd = new FollowPath(maxSpeed,
                maxAccel,
                0.0,
                0.0,
                heading,
                new ArrayList<Translation2d>() {{ }},
                new Pose2d(dest.getX(), dest.getY(), heading),
                dest.getRotation()
        );

        // initialize the follow-path command
        cmd.initialize();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        cmd.execute();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        // we are finished only when follow-path is finished
        return cmd.isFinished();
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        // we are ending, end the follow-path command
        cmd.end(interrupted);
    }

}
