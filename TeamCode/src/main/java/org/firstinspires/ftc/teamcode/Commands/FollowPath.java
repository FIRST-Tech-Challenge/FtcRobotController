package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.Trajectory.State;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.constraint.MecanumDriveKinematicsConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.Utils;

import java.util.List;


// command to follow a path generated from input parameters
public class FollowPath extends CommandBase {

    // Trajectory Configuration
    TrajectoryConfig config;

    // Trajectory
    Trajectory trajectory;

    // trajectory points
    Rotation2d pathStartAngle;
    Pose2d pathEndPose;
    List<Translation2d> pathWaypoints;

    // x,y,theta PID controllers
    PIDController xController, yController, thetaController;

    // time in path
    ElapsedTime pathTime;

    // robot rotation control
    Rotation2d robotEndAngle;
    double endRobotAngle;       // target robot angle in rad
    double startRobotAngle;     // initial robot angle in rad
    double rotationRate;        // desired rotation rate in rad/s

    // constructor
    // MaxSpeed - max robot travel speed (m/s)
    // MaxAccel - max robot travel accel (m/s2)
    // StartSpeed - starting robot speed of path (m/s) - normally 0m/s unless continuing from previous path
    // EndSpeed - ending robot speed of path (m/s) - normally 0m/s unless continuing into subsequent path
    // pathStartAngle - starting angle of generated path (Rotation2d)
    // pathWaypoints - list of waypoints (x,y) (List<Translation2d>)
    // pathEndPose - ending position of path (x,y and angle combined) (Pose2d)
    // robotEndAngle - angle robot faces at end of path (Rotation2d)
   /*public FollowPath(double maxSpeed,
                     double maxAccel,
                     double StartSpeed,
                     double endSpeed,
                     Rotation2d pathStartAngle,
                     Pose2d pathEndPose,
                     Rotation2d robotEndAngle)
   {
       FollowPath (maxSpeed,
       maxAccel,
       StartSpeed,
       endSpeed,
       pathStartAngle,
       List<Translation2d> pathWaypoints,
       pathEndPose,
       robotEndAngle)

   } */


    public FollowPath(double maxSpeed,
                      double maxAccel,
                      double StartSpeed,
                      double endSpeed,
                      Rotation2d pathStartAngle,
                      List<Translation2d> pathWaypoints,
                      Pose2d pathEndPose,
                      Rotation2d robotEndAngle) {

        // add subsystem requirements (if any) - for example:
        addRequirements(RobotContainer.drivesystem);

        // create trajectory configuration
        config = new TrajectoryConfig(maxSpeed, maxAccel);

        // set constraints of trajectory
        config.setReversed(false);
        config.setStartVelocity(StartSpeed);
        config.setEndVelocity(endSpeed);
        config.setKinematics(RobotContainer.drivesystem.GetKinematics());
        config.addConstraint(new MecanumDriveKinematicsConstraint(RobotContainer.drivesystem.GetKinematics(),
                RobotContainer.drivesystem.MAX_SPEED));

        // save positions for use during command initialization
        this.pathStartAngle = pathStartAngle;
        this.pathEndPose = pathEndPose;
        this.pathWaypoints = pathWaypoints;
        this.robotEndAngle = robotEndAngle;

        // configure PID controllers - set PID gains
        xController = new PIDController(5.0, 0.0, 0.0);
        yController = new PIDController(5.0, 0.0, 0.0);
        thetaController = new PIDController(4.0, 0.0, 0.0);

        // configure PID controllers integration limiters - prevents excessive windup of integrated error
        xController.setIntegrationBounds(-0.5, 0.5);
        yController.setIntegrationBounds(-0.5, 0.5);
        thetaController.setIntegrationBounds(-1.0, 1.0);

        // set up timer
        pathTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // get current robot position from odometry
        Pose2d currentPos = RobotContainer.odometry.getCurrentPos();

        // start path at current location of robot - use x,y only!
        // set start angle of path to provided parameter
        Pose2d pathStartPose = new Pose2d(currentPos.getTranslation(),
                pathStartAngle);

        // create trajectory
        trajectory = TrajectoryGenerator.generateTrajectory(pathStartPose,
                pathWaypoints,
                pathEndPose,
                config);

        // set trajectory in odometry for display on dashboard
        RobotContainer.odometry.DisplayTrajectory(trajectory);

        // reset PID controllers - to be ready to run path
        xController.reset();
        yController.reset();
        thetaController.reset();

        // reset path timer
        pathTime.reset();

        // set initial robot rotation target (in rads)
        startRobotAngle = currentPos.getRotation().getRadians();

        // determine rotation rate along path (in rads/s)
        // turn robot by smallest angle (either in +ve or -ve rotation)
        rotationRate = Math.toRadians(Utils.AngleDifference(Math.toDegrees(startRobotAngle),
                robotEndAngle.getDegrees())
        ) / trajectory.getTotalTimeSeconds();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get next reference state from trajectory path
        State targetPathState = trajectory.sample(pathTime.seconds());

        // get current robot position from odometry
        Pose2d currentPos = RobotContainer.odometry.getCurrentPos();

        // determine target robot angle
        double targetRobotAngle = startRobotAngle + (rotationRate * pathTime.seconds());

        // execute PID controllers - error = reference - actual
        // each controller outputs resulting speeds to be given to drive subsystem
        double dX = -xController.calculate(targetPathState.poseMeters.getX() -
                currentPos.getX());
        double dY = -yController.calculate(targetPathState.poseMeters.getY() -
                currentPos.getY());
        double omega = -thetaController.calculate(Utils.AngleDifferenceRads(currentPos.getRotation().getRadians(),targetRobotAngle));

        // go ahead and drive robot
        RobotContainer.drivesystem.FieldDrive(dX, dY, omega);

    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        // command is finished when time in command exceeds expected time of path
        return ( pathTime.seconds() > trajectory.getTotalTimeSeconds() );

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        // this command is done. Remove trajectory in odometry from display on dashboard
        RobotContainer.odometry.DisplayTrajectory(null);
    }

}