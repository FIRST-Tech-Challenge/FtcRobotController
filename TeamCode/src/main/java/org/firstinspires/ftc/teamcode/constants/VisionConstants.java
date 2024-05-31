package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.org.rustlib.vision.AprilTag;

public class VisionConstants {
    public static final int elementDetectionLookBehindFrames = 30;
    public static final AprilTag[] aprilTags = {
            new AprilTag(new Pose3d(8.908, 29.292, 3.897, new Rotation3d(new Rotation2d(), Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(90))), 1),
            new AprilTag(new Pose3d(8.908, 35.292, 3.897, new Rotation3d(new Rotation2d(), Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(90))), 2),
            new AprilTag(new Pose3d(8.908, 41.292, 3.897, new Rotation3d(new Rotation2d(), Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(90))), 3),
            new AprilTag(new Pose3d(8.908, 99.980, 3.897, new Rotation3d(new Rotation2d(), Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(90))), 4),
            new AprilTag(new Pose3d(8.908, 105.980, 3.897, new Rotation3d(new Rotation2d(), Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(90))), 5),
            new AprilTag(new Pose3d(8.908, 111.980, 3.897, new Rotation3d(new Rotation2d(), Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(90))), 6),
    };
}
