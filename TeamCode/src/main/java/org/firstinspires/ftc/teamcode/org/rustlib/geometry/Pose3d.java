package org.firstinspires.ftc.teamcode.org.rustlib.geometry;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class Pose3d extends Vector3d {
    public final Rotation3d rotation;

    public Pose3d(Vector3d vector, Rotation3d rotation) {
        super(vector.x, vector.y, vector.z);
        this.rotation = rotation;
    }

    public Pose3d(Vector3d vector, Rotation2d pitch, Rotation2d roll, Rotation2d yaw) {
        this(vector, new Rotation3d(pitch, roll, yaw));
    }

    public Pose3d(double x, double y, double z, Rotation3d rotation) {
        this(new Vector3d(x, y, z), rotation);
    }

    public Pose3d(double x, double y, double z, Rotation2d pitch, Rotation2d roll, Rotation2d yaw) {
        this(new Vector3d(x, y, z), new Rotation3d(pitch, roll, yaw));
    }

    public Pose3d(Vector3d vector) {
        this(vector, new Rotation3d());
    }

    public Pose3d(double x, double y, double z) {
        this(new Vector3d(x, y, z));
    }

    public Pose3d(Rotation3d rotation) {
        this(new Vector3d(), rotation);
    }

    public Pose3d() {
        this(new Vector3d(), new Rotation3d());
    }

    public static Pose3d average(Pose3d... poses) {
        double xSum = 0;
        double ySum = 0;
        double zSum = 0;
        Rotation2d[] pitch = new Rotation2d[poses.length];
        Rotation2d[] roll = new Rotation2d[poses.length];
        Rotation2d[] yaw = new Rotation2d[poses.length];
        for (int i = 0; i < poses.length; i++) {
            xSum += poses[i].x;
            ySum += poses[i].y;
            zSum += poses[i].z;
            pitch[i] = poses[i].rotation.pitch;
            roll[i] = poses[i].rotation.roll;
            yaw[i] = poses[i].rotation.yaw;
        }
        return new Pose3d(xSum / poses.length, ySum / poses.length, zSum / poses.length, Rotation2d.averageRotations(pitch), Rotation2d.averageRotations(roll), Rotation2d.averageRotations(yaw));
    }

    public Pose3d negate() {
        return new Pose3d(-x, -y, -z, rotation.pitch.negate(), rotation.roll.negate(), rotation.yaw.negate());
    }

    public static Pose3d toPose3d(AprilTagPoseFtc pose) {
        return new Pose3d(pose.x, pose.y, pose.z, new Rotation2d(pose.pitch), new Rotation2d(pose.roll), new Rotation2d(pose.yaw));
    }

    public Pose3d translateX(double x) {
        return translate(x, 0, 0);
    }

    public Pose3d translateY(double y) {
        return translate(0, y, 0);
    }

    public Pose3d translateZ(double z) {
        return translate(0, 0, z);
    }

    public Pose3d translate(double x, double y, double z) {
        return new Pose3d(super.translate(x, y, z), rotation);
    }

    public Pose3d relativeTo(Pose3d pose2) {
        Vector3d newVector = this.pitch(pose2.rotation.pitch.getAngleRadians()).roll(pose2.rotation.roll.getAngleRadians()).yaw(pose2.rotation.yaw.getAngleRadians());
        Vector3d angleVector = new Vector3d();
        return new Pose3d(
                this
                        .pitch(pose2.rotation.pitch.getAngleRadians())
                        .roll(pose2.rotation.roll.getAngleRadians())
                        .yaw(pose2.rotation.yaw.getAngleRadians()),
                new Rotation3d(
                        new Rotation2d(rotation.pitch.getAngleRadians() + pose2.rotation.pitch.getAngleRadians()),
                        new Rotation2d(rotation.roll.getAngleRadians() + pose2.rotation.roll.getAngleRadians()),
                        new Rotation2d(rotation.yaw.getAngleRadians() + pose2.rotation.yaw.getAngleRadians()))
        );
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, rotation.yaw);
    }
}
