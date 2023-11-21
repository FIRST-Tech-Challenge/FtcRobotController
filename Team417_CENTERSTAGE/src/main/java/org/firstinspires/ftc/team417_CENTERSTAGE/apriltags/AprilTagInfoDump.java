package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

/*
    Stores all the info about AprilTags, including location on the field, size, ID, tilt, etc.
    Used in AprilTagDetectionConcept
*/

public class AprilTagInfoDump {

    //Stores the location of the AprilTags in an array
    public static final AprilTag[] aprilTags = new AprilTag[] {
            // Note that these measurements are accurate up to the limits of my ability (up to +- 1/16 inches)
            //    and can double-checked but probably not improved upon
            new AprilTag(7, 5, -72.125, -41.1875, 6.4375),
            new AprilTag(10, 5, -72.125, 41.1875, 6.4375)
    };

    // Finds the april tag with a certain ID to simplify code
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

    //Initializing variables
    public int id;
    public double sideLength;
    public double x;
    public double y;
    public double z;
    public double pitch;
    public double roll;
    public double yaw;

    // Creates an april tag with all the info
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

    // Creates an april tag along the wall where the pixel stacks are
    public AprilTag(int id, double sideLength, double x, double y, double z) {
        this(id, sideLength, x, y, z, 0, 0, 0);
    }
}
