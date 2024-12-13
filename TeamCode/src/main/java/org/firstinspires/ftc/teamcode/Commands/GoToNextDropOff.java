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
public class GoToNextDropOff extends CommandBase {

    // Trajectory Configuration
    TrajectoryConfig config;

    // Trajectory
    Trajectory trajectory;

    // trajectory points
    Rotation2d pathStartAngle;
    Pose2d pathEndPose;
    List<Translation2d> pathWaypoints;

    // ending speed
    double endSpeed;

    // x,y,theta PID controllers
    PIDController xController, yController, thetaController;

    // time in path
    ElapsedTime pathTime;

    // robot rotation control
    Rotation2d robotEndAngle;
    double endRobotAngle;       // target robot angle in rad
    double startRobotAngle;     // initial robot angle in rad
    double rotationRate;        // desired rotation rate in rad/s

    // state variables for manually-implemented robot x & y position PID controllers
    double x_ierror;
    double y_ierror;

    // destination decrement value
    static double x_dest_decrement;

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


    public GoToNextDropOff(double maxSpeed,
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

        // save ending speed of robot
        this.endSpeed = endSpeed;

        // configure PID controllers - set PID gains
        xController = new PIDController(15.0, 2.00, 0.0);
        yController = new PIDController(15.0, 2.00, 0.0);
        thetaController = new PIDController(10.0, 0.05, 0.0);

        // configure PID controllers integration limiters - prevents excessive windup of integrated error
        xController.setIntegrationBounds(-10.0, 10.0);
        yController.setIntegrationBounds(-10.0, 10.0);
        thetaController.setIntegrationBounds(-5.0, 5.0);

        // set up timer
        pathTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    // initialize destination position decrement
    public static void initializeDestinationDecrement() {
        x_dest_decrement = 0.0;
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

        // determine new destination
        Pose2d NewDestination = new Pose2d(pathEndPose.getX()+x_dest_decrement,
                                pathEndPose.getY(),
                                pathEndPose.getRotation());

        // create trajectory
        // use try-catch block in case trajectory cannot be formed using given points
        // if trajectory cannot formed then set to null. Command will detect and exit
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(pathStartPose,
                    pathWaypoints,
                    NewDestination,
                    config);
        }
        catch (Exception e) {
            trajectory = null;
        }

        // set trajectory in odometry for display on dashboard
        RobotContainer.odometry.DisplayTrajectory(trajectory);

        // reset PID controllers - to be ready to run path
        xController.reset();
        yController.reset();
        thetaController.reset();

        // reset manually-implemented x and y PID controller state values
        x_ierror=0.0;
        y_ierror = 0.0;

        // reset path timer
        pathTime.reset();

        // set initial robot rotation target (in rads)
        startRobotAngle = currentPos.getRotation().getRadians();

        // determine rotation rate along path (in rads/s)
        // turn robot by smallest angle (either in +ve or -ve rotation)
        if (trajectory!=null&& trajectory.getTotalTimeSeconds() !=0.0)
            rotationRate = Math.toRadians(Utils.AngleDifference(Math.toDegrees(startRobotAngle),
                robotEndAngle.getDegrees())) / trajectory.getTotalTimeSeconds();
        else
            rotationRate = 0.0;

        // decrement destination for next time
        if(RobotContainer.isRedAlliance)
            x_dest_decrement -=0.05;
        else
            x_dest_decrement +=0.05;

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // if we have valid path then execute. Otherwise do nothing.
        if (trajectory!=null) {

            // get next reference state from trajectory path
            State targetPathState = trajectory.sample(pathTime.seconds());

            // get current robot position from odometry
            Pose2d currentPos = RobotContainer.odometry.getCurrentPos();

            // determine target robot angle
            double targetRobotAngle;
            if (pathTime.seconds() < trajectory.getTotalTimeSeconds())
                targetRobotAngle = startRobotAngle + (rotationRate * pathTime.seconds());
            else
                targetRobotAngle = startRobotAngle + (rotationRate * trajectory.getTotalTimeSeconds());

            // commented out for now.  Will try using manual implementation of PID
            // execute PID controllers - error = reference - actual
            // each controller outputs resulting speeds to be given to drive subsystem
            //double dX = -xController.calculate(targetPathState.poseMeters.getX() -
            //        currentPos.getX());

            // manual implementation of x-position PI controller
            double error = targetPathState.poseMeters.getX() - currentPos.getX();
            x_ierror += 0.60 * error;
            if (Math.abs(error) > 0.03)
                x_ierror = 0.0;
            double dX = 8.0 * error + x_ierror;

            // commented out for now.  Will try using manual implementation of PID
            // double dY = -yController.calculate(targetPathState.poseMeters.getY() -
            //                currentPos.getY());

            // manual implementation of y-position PI controller
            double y_error = targetPathState.poseMeters.getY() - currentPos.getY();
            y_ierror += 0.60 * y_error;
            if (Math.abs(y_error) > 0.03)
                y_ierror = 0.0;
            double dY = 8.0 * y_error + y_ierror;

            // robot rotation controller
            double omega = -thetaController.calculate(Utils.AngleDifferenceRads(currentPos.getRotation().getRadians(), targetRobotAngle));

            // Dashboard values - used for testing purposes. Keep for now if needed in future
            //        RobotContainer.DBTelemetry.addData("Path Error X ",
            //                targetPathState.poseMeters.getX() - currentPos.getX());
            //        RobotContainer.DBTelemetry.addData("Path Error Y ",
            //                targetPathState.poseMeters.getY() - currentPos.getY());
            //        RobotContainer.DBTelemetry.addData("Path Error Theta ",
            //                Utils.AngleDifferenceRads(currentPos.getRotation().getRadians(),targetRobotAngle));
            //
            //        RobotContainer.DBTelemetry.addData("X Control Out ",
            //                dX);
            //        RobotContainer.DBTelemetry.addData("Y Control Out ",
            //                dY);
            //        RobotContainer.DBTelemetry.addData("Z Control Out ",
            //                omega);
            //        RobotContainer.DBTelemetry.update();

            // go ahead and drive robot
            RobotContainer.drivesystem.FieldDrive(dX, dY, omega);

        }   // end if we have valid trajectory to follow
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        // command is finished if no valid path or when time in command exceeds expected time of path
        return ( trajectory==null ||
                (pathTime.seconds() > (trajectory.getTotalTimeSeconds()+0.25)) );

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

        // this command is done. Remove trajectory in odometry from display on dashboard
        RobotContainer.odometry.DisplayTrajectory(null);

        // if end speed is 0m/s, then stop drive to ensure robot drive is not left 'creeping'
        if (endSpeed==0.0)
            RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0);

    }

}