package org.firstinspires.ftc.teamcode.org.rustlib.vision;

import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose3d;

import java.util.ArrayList;

public class AprilTag {
    private static ArrayList<AprilTag> tags = new ArrayList<>();
    public final Pose3d pose;
    public final int id;

    public AprilTag(Pose3d pose, int id) {
        this.pose = pose;
        this.id = id;
        tags.add(this);
    }

    public static AprilTag getTag(int id) {
        for (AprilTag tag : tags) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }
}
