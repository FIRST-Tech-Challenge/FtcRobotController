package org.firstinspires.ftc.robot;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;

import org.firstinspires.ftc.robot_utilities.Vals;

public class DriveTrain {

    public Motor driveLeft, driveRight;

    private DifferentialDriveKinematics m_kinematics;
    private SimpleMotorFeedforward ff;

    public DriveTrain(Motor driveLeft, Motor driveRight) {
        this.driveLeft = driveLeft;
        this.driveRight = driveRight;

        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
//        driveLeft.setInverted(true);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);
        driveLeft.setFeedforwardCoefficients(1, 3);
        driveRight.setFeedforwardCoefficients(1, 3);
        driveLeft.setPositionCoefficient(0.05);
        driveRight.setPositionCoefficient(0.05);

        m_kinematics = new DifferentialDriveKinematics(Vals.TRACK_WIDTH_METERS);
        ff = new SimpleMotorFeedforward(Vals.drive_ks, Vals.drive_kv);
    }

    public DriveTrain(Motor driveLeft, Motor driveRight, Motor.RunMode runMode) {
        this(driveLeft, driveRight);

        this.driveLeft.setRunMode(runMode);
        this.driveRight.setRunMode(runMode);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        driveLeft.set(-leftSpeed);
        driveRight.set(rightSpeed);
    }

    public void setSpeedPositiveForward(double leftSpeed, double rightSpeed) {
        driveLeft.set(leftSpeed);
        driveRight.set(-rightSpeed);
    }

    public void setSpeed(ChassisSpeeds speeds, TelemetryPacket packet) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
//        wheelSpeeds.normalize();

        packet.put("Left Meters Speed", wheelSpeeds.leftMetersPerSecond);
        packet.put("Right Meters Speed", wheelSpeeds.rightMetersPerSecond);


        double leftSpeed = ff.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightSpeed = ff.calculate(wheelSpeeds.rightMetersPerSecond);
//        double rightSpeed = wheelSpeeds.rightMetersPerSecond/Vals.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;

        packet.put("Left Before Speed", leftSpeed);
        packet.put("Right Before Speed", rightSpeed);

//        leftSpeed = Math.min(1, Math.max(-1, leftSpeed));
//        rightSpeed = Math.min(1, Math.max(-1, rightSpeed));
//
//        packet.put("Left Speed", leftSpeed);
//        packet.put("Right Speed", rightSpeed);

        setSpeedPositiveForward(leftSpeed, rightSpeed);
    }

    public void setTargetPosition(int targetPosition) {
        driveLeft.setTargetPosition(-targetPosition);
        driveLeft.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(int leftTargetPosition, int rightTargetPosition) {
        driveLeft.setTargetPosition(-leftTargetPosition);
        driveLeft.setTargetPosition(rightTargetPosition);
    }

    public int[] getPosition() {
        return new int[]{driveLeft.getCurrentPosition(), driveRight.getCurrentPosition()};
    }

    public double[] getDistance() {
        return new double[]{driveLeft.getDistance(), -driveRight.getDistance()};
    }

    public double getAverageDistance() {
        return (driveLeft.getDistance() - driveRight.getDistance())/2;
    }

    public double[] getRevolutions() {
        return new double[]{driveLeft.encoder.getRevolutions(), driveRight.encoder.getRevolutions()};
    }

    public void resetEncoders() {
        driveLeft.resetEncoder();
        driveRight.resetEncoder();
    }



}
