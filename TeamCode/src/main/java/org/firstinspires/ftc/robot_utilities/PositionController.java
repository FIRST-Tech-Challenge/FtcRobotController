package org.firstinspires.ftc.robot_utilities;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;

public class PositionController {

    public DifferentialDriveOdometry odometry;

    private Pose2d startPose;
    private PIDController pidDrive;
    private RotationController rotationController;

    public PositionController(Pose2d currentPose, RotationController rotationController) {
        pidDrive = new PIDController(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        pidDrive.setTolerance(Vals.drive_tolerance);

        this.rotationController = rotationController;
        this.startPose = currentPose;
        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), currentPose);
    }

    /**
     * Update current position
     * @param driveTrainDistance
     */
    public void update(double[] driveTrainDistance) {
        double leftDistanceInch = driveTrainDistance[0] / Vals.TICKS_PER_INCH_MOVEMENT;
        double rightDistanceInch = driveTrainDistance[1] / Vals.TICKS_PER_INCH_MOVEMENT;
        odometry.update(new Rotation2d(rotationController.getAngleRadians()), leftDistanceInch, rightDistanceInch);

    }

    /**
     * Functional code with no use outside of tuning
     */
    public void updatePID() {
        pidDrive.setPID(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        pidDrive.setTolerance(Vals.drive_tolerance);
    }

    /**
     * functional code - self explanatory
     * @param a
     * @param b
     * @return
     */
    private double getDistance(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private double getRotationToPoint(Pose2d target, Pose2d o) {
        double dy = target.getY() - o.getY();
        double dx = -target.getX() + o.getX();

//        if(dx == 0) return 0;

        return Math.toDegrees(Math.atan2(dx, dy));
    }

    public void reset() {
        pidDrive.reset();
        startPose = odometry.getPoseMeters();
        rotationController.resetAngle();
    }

    public double[] goto_pose(Pose2d targetPose, TelemetryPacket packet) {
        double targetDistance = getDistance(targetPose, startPose);
        double currentDistance = getDistance(odometry.getPoseMeters(), startPose);
        double rotationToPoint = getRotationToPoint(targetPose, startPose);

        double leftSpeed = 0;
        double rightSpeed = 0;

        double power = 0;
        double power2 = 0;

        if(currentDistance < 1) {
            power = rotationController.rotate(rotationToPoint);
        } else if(targetDistance - currentDistance > 1) {
            power = rotationController.rotate(rotationToPoint);
            power2 = pidDrive.calculate(currentDistance, targetDistance);
        } else {
            power = rotationController.rotate(rotationToPoint);
            power2 = pidDrive.calculate(currentDistance, targetDistance);
        }

        leftSpeed += power;
        rightSpeed -= power;

        leftSpeed -= power2;
        rightSpeed -= power2;

        packet.put("Rotation to point", rotationToPoint);
        packet.put("Target Distance", targetDistance);
        packet.put("Current Distance", currentDistance);
        packet.put("Position Power", power2);
        packet.put("Rotation Power", power);

        return new double[]{leftSpeed, rightSpeed};
    }



}
