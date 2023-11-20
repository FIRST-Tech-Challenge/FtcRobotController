package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

/*
    Stores all the info about AprilTags, including location on the field, size, ID, tilt, etc.
    Used in AprilTagDetectionConcept
*/

public class AprilTagInfoDump {
    public static final AprilTag[] aprilTags = new AprilTag[] {
            // Note that these measurements are not accurate (up to +- 2 inches) and can be improved on
            new AprilTag(7, 5, -72, -42, 5.5),
            new AprilTag(10, 5, -72, 42, 1)
    };

    public static AprilTag findTagWithId(int id) {
        for (AprilTag aprilTag : aprilTags) {
            if (aprilTag.id == id) {
                return aprilTag;
            }
        }
        return null;
    }
}

// Stores data about April Tags
class AprilTag {
    public int id;
    public double sideLength;
    public double x;
    public double y;
    public double z;
    public double pitch;
    public double roll;
    public double yaw;

    public AprilTag(int id, double sideLength, double x, double y, double z, double pitch, double roll, double yaw) {
        this.id = id;
        this.sideLength = sideLength;
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    public AprilTag(int id, double sideLength, double x, double y, double z) {
        this(id, sideLength, x, y, z, 0, 0, 0);
    }
}
