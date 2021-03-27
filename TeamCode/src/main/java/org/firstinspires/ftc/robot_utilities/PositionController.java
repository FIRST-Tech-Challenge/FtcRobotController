package org.firstinspires.ftc.robot_utilities;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

public class PositionController {

    double targetDistance = 0;
    Pose2d startPose;
    private PIDController pidDrive;

    public PositionController(Pose2d currentPose) {
        pidDrive = new PIDController(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        pidDrive.setTolerance(Vals.drive_tolerance);

        this.startPose = currentPose;
    }

    private double getDistance(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }


    public void setTarget(Pose2d currentPose) {
        pidDrive.reset();
        targetDistance = 0;
        startPose = currentPose;
    }

    public void setTarget(Pose2d currentPose, Pose2d targetPose) {
        pidDrive.reset();
        targetDistance = getDistance(targetPose, currentPose);
        startPose = currentPose;
    }

    public double goto_pos(Pose2d currentPose) {
        double currentDistance = getDistance(currentPose, startPose);
        return pidDrive.calculate(currentDistance, targetDistance);
    }



}
