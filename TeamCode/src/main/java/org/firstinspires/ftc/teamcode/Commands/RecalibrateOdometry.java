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
    private static final double redAllianceXwall = -150.0;
    private static final double redAllianceYwall = -150.0;
    private static final double blueAllianceXwall = 150.0;
    private static final double blueAllianceYwall = 150.0;

    // Tolerance values for position and angle
    private static final double positionTolerance = 0.1;
    private static final double angleTolerance = 0.0;

    // Maximum X and Y values for the Red and Blue Alliance
    private static final double redAllianceMaxX = -150.0;
    private static final double redAllianceMaxY = -150.0;
    private static final double blueAllianceMaxX = 150.0;
    private static final double blueAllianceMaxY = 150.0;

    // Angles the robot should be facing the wall
    private static final double redAllianceXwallAngle = 0.0;
    private static final double redAllianceYwallAngle = 90.0;
    private static final double blueAllianceXwallAngle = 180.0;
    private static final double blueAllianceYwallAngle = -90.0;

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
        if (inNetZone()) {
            // Facing X Wall
            if (facingXWall()) {
                if (isRedAlliance) {
                    RobotContainer.odometry.setCurrentPos(new Pose2d(redAllianceXwall, currentPos.getY(), new Rotation2d(redAllianceXwallAngle)));
                } else {
                    RobotContainer.odometry.setCurrentPos(new Pose2d(blueAllianceXwall, currentPos.getY(), new Rotation2d(blueAllianceXwallAngle)));
                }
            }

            // Facing Y Wall
            if (facingYWall()) {
                if (isRedAlliance) {
                    RobotContainer.odometry.setCurrentPos(new Pose2d(currentPos.getX(), redAllianceYwall, new Rotation2d(redAllianceYwallAngle)));
                } else {
                    RobotContainer.odometry.setCurrentPos(new Pose2d(currentPos.getX(), blueAllianceYwall, new Rotation2d(blueAllianceYwallAngle)));
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
            return Math.abs(angle - redAllianceXwallAngle) <= angleTolerance;
        } else {
            return Math.abs(angle - blueAllianceXwallAngle) <= angleTolerance;
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
            return Math.abs(angle - redAllianceYwallAngle) <= angleTolerance;
        } else {
            return Math.abs(angle - blueAllianceYwallAngle) <= angleTolerance;
        }
    }

    /**
     * Checks if the robot is in the NetZone.
     *
     * @return true if the robot is in the NetZone, false otherwise.
     */
    private boolean inNetZone() {
        boolean inTheZone = false;
        // Check if we are in the NetZone
        if (isRedAlliance) {
            // Check if we are in the Red Alliance NetZone
            inTheZone = Math.abs(currentPos.getX() - redAllianceMaxX) <= positionTolerance && Math.abs(currentPos.getY() - redAllianceMaxY) <= positionTolerance;
        } else {
            // Check if we are in the Blue Alliance NetZone
            inTheZone = Math.abs(currentPos.getX() - blueAllianceMaxX) <= positionTolerance && Math.abs(currentPos.getY() - blueAllianceMaxY) <= positionTolerance;
        }
        return inTheZone;
    }
}