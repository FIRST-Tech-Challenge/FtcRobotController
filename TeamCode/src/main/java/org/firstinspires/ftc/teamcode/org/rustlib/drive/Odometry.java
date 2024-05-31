package org.firstinspires.ftc.teamcode.org.rustlib.drive;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Subsystem;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.org.rustlib.hardware.Encoder;
import org.firstinspires.ftc.teamcode.org.rustlib.hardware.PairedEncoder;
import org.firstinspires.ftc.teamcode.org.rustlib.rustboard.Rustboard;

import java.util.function.DoubleSupplier;

public class Odometry extends Subsystem {
    public final Encoder rightEncoder;
    public final Encoder leftEncoder;
    public final Encoder backEncoder;
    private int lastRight = 0;
    private int lastLeft = 0;
    private int lastBack = 0;
    private double lastIMUHeading = 0;
    private Pose2d pose;
    private final DoubleSupplier imuHeading;

    public Odometry(Builder builder) {
        pose = new Pose2d();
        rightEncoder = builder.rightEncoder;
        leftEncoder = builder.leftEncoder;
        backEncoder = builder.backEncoder;
        imuHeading = builder.imuHeading;
        resetEncoders();
    }

    public interface LeftEncoder {
        RightEncoder defineLeftEncoder(PairedEncoder encoder);
    }

    public interface RightEncoder {
        BackEncoder defineRightEncoder(PairedEncoder encoder);
    }

    public interface BackEncoder {
        TrackWidth defineBackEncoder(PairedEncoder encoder);
    }

    public interface TrackWidth {
        VerticalDistance setTrackWidth(double width);
    }

    public interface VerticalDistance {
        InPerTick setVerticalDistance(double distance);
    }

    public interface InPerTick {
        Builder setInPerTick(double inPerTick);
    }

    public static class Builder implements LeftEncoder, RightEncoder, BackEncoder, TrackWidth, VerticalDistance, InPerTick {
        private Encoder rightEncoder;
        private Encoder leftEncoder;
        private Encoder backEncoder;
        private double trackWidth;
        private double verticalDistance;
        private double inPerTick;
        private DoubleSupplier imuHeading = null;

        private Builder() {

        }

        @Override
        public RightEncoder defineLeftEncoder(PairedEncoder encoder) {
            leftEncoder = encoder;
            return this;
        }

        @Override
        public BackEncoder defineRightEncoder(PairedEncoder encoder) {
            rightEncoder = encoder;
            return this;
        }

        @Override
        public TrackWidth defineBackEncoder(PairedEncoder encoder) {
            backEncoder = encoder;
            return this;
        }

        @Override
        public VerticalDistance setTrackWidth(double width) {
            trackWidth = width;
            return this;
        }

        @Override
        public InPerTick setVerticalDistance(double distance) {
            verticalDistance = distance;
            return this;
        }

        @Override
        public Builder setInPerTick(double inPerTick) {
            this.inPerTick = inPerTick;
            return this;
        }

        public Builder useIMU(DoubleSupplier imuHeading) {
            this.imuHeading = imuHeading;
            return this;
        }

        public Odometry build() {
            return new Odometry(this);
        }
    }

    public static LeftEncoder getBuilder() {
        return new Builder();
    }

    public void setPosition(Pose2d pose) {
        this.pose = pose;
    }

    private Pose2d delta() {
        int currentRightTicks = rightEncoder.getPosition();
        int currentLeftTicks = leftEncoder.getPosition();
        int currentBackTicks = backEncoder.getPosition();

        double deltaRight = currentRightTicks - lastRight;
        double deltaLeft = currentLeftTicks - lastLeft;
        double deltaBack = currentBackTicks - lastBack;

        double deltaHeading;
        if (imuHeading == null) {
            deltaHeading = DriveConstants.Odometry.inPerTick * (deltaRight - deltaLeft) / DriveConstants.Odometry.trackWidth;
        } else {
            double currentHeading = imuHeading.getAsDouble();
            deltaHeading = currentHeading - lastIMUHeading;
            lastIMUHeading = currentHeading;
        }

        double deltaXDrive;
        double deltaYDrive;

        double deltaXStrafe;
        double deltaYStrafe;

        if (deltaHeading == 0.0) {
            deltaXDrive = 0;
            deltaYDrive = DriveConstants.Odometry.inPerTick * deltaRight;

            deltaXStrafe = DriveConstants.Odometry.inPerTick * deltaBack;
            deltaYStrafe = 0;
        } else {
            double par0Radius = DriveConstants.Odometry.inPerTick * deltaRight / deltaHeading;
            double par1Radius = DriveConstants.Odometry.inPerTick * deltaLeft / deltaHeading;

            double driveRadius = (par0Radius + par1Radius) / 2;

            deltaXDrive = -driveRadius * (1 - Math.cos(deltaHeading));
            deltaYDrive = driveRadius * Math.sin(deltaHeading);

            double strafeRadius = DriveConstants.Odometry.inPerTick * deltaBack / deltaHeading - DriveConstants.Odometry.verticalDistance;

            deltaXStrafe = strafeRadius * Math.sin(deltaHeading);
            deltaYStrafe = strafeRadius * (1 - Math.cos(deltaHeading));
        }
        lastRight = currentRightTicks;
        lastLeft = currentLeftTicks;
        lastBack = currentBackTicks;
        return new Pose2d(deltaXDrive + deltaXStrafe, deltaYDrive + deltaYStrafe, new Rotation2d(deltaHeading));
    }

    public Pose2d update() {
        Rustboard.setNodeValue(".right odometry encoder", rightEncoder.getPosition());
        Rustboard.setNodeValue(".left odometry encoder", leftEncoder.getPosition());
        Rustboard.setNodeValue(".back odometry encoder", backEncoder.getPosition());
        Pose2d delta = delta();
        pose = pose.add(new Pose2d(delta.rotate(pose.rotation.getAngleRadians()), new Rotation2d(delta.rotation.getAngleRadians())));
        return pose;
    }

    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
        backEncoder.reset();
    }

    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void periodic() {
        update();
    }
}
