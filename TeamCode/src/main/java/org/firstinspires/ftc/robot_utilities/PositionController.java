package org.firstinspires.ftc.robot_utilities;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;

public class PositionController {

    public DifferentialDriveOdometry odometry;

    private Pose2d startPose;
    private Pose2d m_poseError = new Pose2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private PIDController pidDrive;
    private RotationController rotationController;

    double b;
    double zeta;

    public PositionController(Pose2d currentPose,
                              RotationController rotationController,
                              double b,
                              double zeta) {
        pidDrive = new PIDController(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        pidDrive.setTolerance(Vals.drive_tolerance);

        this.b = b;
        this.zeta = zeta;

        this.rotationController = rotationController;
        this.startPose = currentPose;
        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), currentPose);
    }

    public PositionController(Pose2d currentPose, RotationController rotationController) {
        this(currentPose, rotationController, 2.0, 0.7);
    }

    /**
     * Sets the pose error which is considered tolerable for use with atReference().
     * @param poseTolerance Pose error which is tolerable.
     */
    public void setTolerance(Pose2d poseTolerance) {
        this.m_poseTolerance = poseTolerance;
    }

    /** Returns true if the pose error is within tolerance of the reference. */
    public boolean atReference() {
        final Translation2d eTranslate = m_poseError.getTranslation();
        final Rotation2d eRotate = m_poseError.getRotation();
        final Translation2d tolTranslate = m_poseTolerance.getTranslation();
        final Rotation2d tolRotate = m_poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
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

    /**
     * Returns sin(x) / x.
     *
     * @param x Value of which to take sinc(x).
     */
    @SuppressWarnings("ParameterName")
    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
            return 1.0 - 1.0 / 6.0 * x * x;
        } else {
            return Math.sin(x) / x;
        }
    }

    public void reset() {
        pidDrive.reset();
        startPose = odometry.getPoseMeters();
        rotationController.resetAngle();
    }

    public double[] goto_pose(Pose2d targetPose,
                              double linearVelocityRefMeters,
                              double angularVelocityRefRadiansPerSecond,
                              TelemetryPacket packet) {

        Pose2d currentPose = odometry.getPoseMeters();

        m_poseError = targetPose.relativeTo(currentPose);

        final double eX = m_poseError.getX();
        final double eY = m_poseError.getY();
        final double eTheta = m_poseError.getRotation().getRadians();
        final double vRef = linearVelocityRefMeters;
        final double omegaRef = angularVelocityRefRadiansPerSecond;

        double k = 2.0 * zeta * Math.sqrt(Math.pow(omegaRef, 2) + b * Math.pow(vRef, 2));

        double linearVelocityInMeters = vRef * m_poseError.getRotation().getCos() + k * eX;
        double angularVelocityInRadiansPerSecond = omegaRef + k * eTheta + b * vRef * sinc(eTheta) * eY;
//        double targetDistance = getDistance(targetPose, startPose);
//        double currentDistance = getDistance(odometry.getPoseMeters(), startPose);
//        double rotationToPoint = getRotationToPoint(targetPose, startPose);
//
//        double leftSpeed = 0;
//        double rightSpeed = 0;
//
//        double power = 0;
//        double power2 = 0;
//
//        power = rotationController.rotate(rotationToPoint);
//        power2 = pidDrive.calculate(currentDistance, targetDistance);
//
//        leftSpeed += power;
//        rightSpeed -= power;
//
//        leftSpeed -= power2;
//        rightSpeed -= power2;

//        packet.put("Rotation to point", rotationToPoint);
//        packet.put("Target Distance", targetDistance);
//        packet.put("Current Distance", currentDistance);
//        packet.put("Position Power", power2);
//        packet.put("Rotation Power", power);


        packet.put("eX", eX);
        packet.put("eY", eY);
        packet.put("eTheta", eTheta);
        packet.put("Linear Meters Velocity", linearVelocityInMeters);
        packet.put("Angular Degrees Velocity", Math.toDegrees(angularVelocityInRadiansPerSecond));

        if(atReference()) {
            return new double[]{0, 0};
        }

        return convert2Robot(linearVelocityInMeters, angularVelocityInRadiansPerSecond);
    }

    private double[] convert2Robot(double linearVelocityRefMeters,
                                   double angularVelocityRefRadiansPerSecond) {
        double linearVelocityRobot = Math.max(1, linearVelocityRefMeters / Vals.MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
        double angularVelocityRobot = Math.max(1, angularVelocityRefRadiansPerSecond / Vals.MAX_ANGULAR_VELOCITY_RADIANS);

        return new double[]{linearVelocityRobot, angularVelocityRobot};
    }



}
