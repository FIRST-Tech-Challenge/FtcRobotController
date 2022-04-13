package org.firstinspires.ftc.teamcode.AutonomousDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousBasicDrive extends LinearOpMode {
    protected HardwareRobot robot;
    protected ElapsedTime timer;

    AutonomousBasicDrive(HardwareRobot robot) {
        timer = new ElapsedTime();
        this.robot = robot;
    }

    // This function should be implemented to perform the actions required for the game.
    public abstract void performRobotActions();

    /**
     * @param targetAngle - Angle to rotate to in degrees
     */
    public void rotateToAngle(double targetAngle) {
        boolean targetAngleReached = false;
        while (opModeIsActive() && !targetAngleReached) {
            robot.readBulkData();
            targetAngleReached = robot.rotateTowardsAngle(targetAngle);
            performRobotActions();
        }
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeadingForTime(double speed, double rotateSpeed, double driveAngle, double headingAngle, int driveTime, boolean stopWhenDone) {
        double endTime = timer.milliseconds() + driveTime;
        while (opModeIsActive() && (timer.milliseconds() <= endTime)) {
            robot.readBulkData();
            robot.driveAtHeading(speed, rotateSpeed, driveAngle, headingAngle);
            performRobotActions();
        }
        if(stopWhenDone) {
            robot.driveTrainMotorsZero();
        }
    }
}
