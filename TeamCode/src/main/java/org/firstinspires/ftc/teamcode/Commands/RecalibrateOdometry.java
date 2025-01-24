package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Command to recalibrate the odometry of the robot.
 */
public class RecalibrateOdometry extends CommandBase {

    // Constants for the Red and Blue Alliance wall positions and angles
    private static final double redAllianceXsub = -0.77;
    private static final double redAllianceYsub = -0.5;
    private static final double blueAllianceXsub = 0.5;
    private static final double blueAllianceYsub = 0.77;

    // Tolerance values for position and angle
    private static final double positionTolerance = 0.2;
    private static final double angleTolerance = 0.0;

    // Maximum X and Y values for the Red and Blue Alliance
    private static final double redAllianceMaxX = -0.77;
    private static final double redAllianceMaxY = -0.5;
    private static final double blueAllianceMaxX = 0.5;
    private static final double blueAllianceMaxY = 0.77;

    // Angles the robot should be facing the wall
    private static final double redAllianceXsubAngle = RobotContainer.RedStartAngle;
    private static final double redAllianceYsubAngle = 0.0;
    private static final double blueAllianceXsubAngle = 180;
    private static final double blueAllianceYsubAngle = RobotContainer.BlueStartAngle;

    // Determine if the robot is on the Red Alliance
    private static final boolean isRedAlliance = RobotContainer.isRedAlliance();

    // Current position of the robot
    Pose2d currentPos;

    //finished updating the robot's position based on its current position and alliance color.
    boolean finishedUpdating = false;

    /**
     * Constructor for the recalibrateOdometry command.
     * Adds the odometry subsystem as a requirement.
     */
    public RecalibrateOdometry() {
        addRequirements(RobotContainer.odometry);
    }

    /**
     * Called once when the command is started.
     * Initializes the current position and recalibrates the robot's position based on the alliance color.
     */
    @Override
    public void initialize() {
        currentPos = RobotContainer.odometry.getCurrentPos();
        updatePosition();
        RobotContainer.DBTelemetry.addData("Pressed","Depressed");
        RobotContainer.DBTelemetry.update();
    }

    /**
     * Called periodically while the command is active.
     */
    @Override
    public void execute() {
        // No periodic actions required for this command
    }

    /**
     * Determines if the command is finished.
     *
     * @return false as this command does not have a termination condition.
     */
    @Override
    public boolean isFinished() {
        return finishedUpdating;
    }

    /**
     * Called once when the command is finished.
     *
     * @param interrupted whether the command was interrupted/canceled.
     */
    @Override
    public void end(boolean interrupted) {
        // No cleanup actions required for this command
    }

    /**
     * Updates the robot's position based on its current position and alliance color.
     */
    private void updatePosition() {
        if (inSubZone()) {
            // Facing X Wall
            if (facingXWall()) {
               RobotContainer.DBTelemetry.addData("Fasing X Wall","Yes");
               RobotContainer.DBTelemetry.update();
                if (isRedAlliance) {
                    RobotContainer.DBTelemetry.addData("Y position set to ", redAllianceXsub);
                    RobotContainer.DBTelemetry.update();
                    RobotContainer.odometry.setCurrentPos(new Pose2d(redAllianceXsub, currentPos.getY(), new Rotation2d(redAllianceXsubAngle)));
                } else {
                    RobotContainer.DBTelemetry.addData("Y position set to ", blueAllianceXsub);
                    RobotContainer.DBTelemetry.update();
                    RobotContainer.odometry.setCurrentPos(new Pose2d(blueAllianceXsub, currentPos.getY(), new Rotation2d(blueAllianceXsubAngle)));
                }
            }

            // Facing Y Wall
            if (facingYWall()) {
                RobotContainer.DBTelemetry.addData("Fasing Y Wall","Yes");
                RobotContainer.DBTelemetry.update();
                if (isRedAlliance) {
                    RobotContainer.DBTelemetry.addData("X position set to ", redAllianceYsub);
                    RobotContainer.DBTelemetry.update();
                    RobotContainer.odometry.setCurrentPos(new Pose2d(currentPos.getX(), redAllianceYsub, new Rotation2d(redAllianceYsubAngle)));
                } else {
                    RobotContainer.DBTelemetry.addData("X position set to ", blueAllianceYsub);
                    RobotContainer.DBTelemetry.update();
                    RobotContainer.odometry.setCurrentPos(new Pose2d(currentPos.getX(), blueAllianceYsub, new Rotation2d(blueAllianceYsubAngle)));
                }
            }
        }
        finishedUpdating = true;
    }

    /**
     * Checks if the robot is facing the X wall.
     *
     * @return true if the robot is facing the X wall, false otherwise.
     */
    private boolean facingXWall() {
        double angle = currentPos.getHeading();
        if (isRedAlliance) {
            return Math.abs(angle - redAllianceXsubAngle) <= angleTolerance;
        } else {
            return Math.abs(angle - blueAllianceXsubAngle) <= angleTolerance;
        }
    }

    /**
     * Checks if the robot is facing the Y wall.
     *
     * @return true if the robot is facing the Y wall, false otherwise.
     */
    private boolean facingYWall() {
        double angle = currentPos.getHeading();
        if (isRedAlliance) {
            return Math.abs(angle - redAllianceYsubAngle) <= angleTolerance;
        } else {
            return Math.abs(angle - blueAllianceYsubAngle) <= angleTolerance;
        }
    }

    /**
     * Checks if the robot is in the NetZone.
     *
     * @return true if the robot is in the NetZone, false otherwise.
     */
    private boolean inSubZone() {
        boolean inTheZone = false;
        // Check if we are in the NetZone
        if (isRedAlliance) {
            // Check if we are in the Red Alliance NetZone
            inTheZone = Math.abs(currentPos.getX() - redAllianceMaxX) <= positionTolerance || Math.abs(currentPos.getY() - redAllianceMaxY) <= positionTolerance;
        } else {
            // Check if we are in the Blue Alliance NetZone
            inTheZone = Math.abs(currentPos.getX() - blueAllianceMaxX) <= positionTolerance || Math.abs(currentPos.getY() - blueAllianceMaxY) <= positionTolerance;
        }
        return inTheZone;
        //return true;
    }
}