package org.firstinspires.ftc.teamcode.classes;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.pid.DeadzonePID;

public class Drive {

    private final Motor frontLeft, frontRight, backLeft, backRight;
    private double maxOutput = 1;
    private final DeadzonePID headingDeadzone;
    private final AngleController headingController;

    private double turnSpeed;
    private double theta;

    private Pose2d currentPose;

    public Drive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        headingDeadzone = new DeadzonePID(new PIDCoefficientsEx(0, 0, 0, 1, 1, 0), 2);
        headingController = new AngleController(headingDeadzone);
    }

    public Drive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, PIDCoefficientsEx turningCoeffs) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        headingDeadzone = new DeadzonePID(turningCoeffs, Math.toRadians(2));
        headingController = new AngleController(headingDeadzone);
        currentPose = new Pose2d(0, 0, 0);
    }

    public void setMaxSpeed(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;

        driveWithMotorPowers(
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        );
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        driveRobotCentric(
                input.getX(),
                input.getY(),
                turnSpeed
        );
    }

    public void drivePointCentric(double strafeSpeed, double forwardSpeed,
                                  double gyroAngle,
                                  Vector2d target, Pose2d currentPose, double angleOffset) {

        this.currentPose = currentPose;

        Vector2d difference = target.subtract(currentPose.getVector());

        theta = difference.getAngle() + Math.toRadians(angleOffset);

        turnSpeed = headingController.calculate(theta, currentPose.getTheta());

        double turnError = Math.abs(theta = currentPose.getTheta());
//
//        strafeSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));
//        forwardSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));

        driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                -turnSpeed,
                gyroAngle
        );
    }



    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    public double getTurnTarget() {
        return theta;
    }



    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        this.frontLeft
                .set(frontLeftSpeed * maxOutput);
        this.frontRight
                .set(frontRightSpeed * maxOutput);
        this.backLeft
                .set(backLeftSpeed * maxOutput);
        this.backRight
                .set(backRightSpeed * maxOutput);
    }

}
