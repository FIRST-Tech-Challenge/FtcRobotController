package org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses;

// Stores data about April Tags
public class AprilTagInfo {

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
    public AprilTagInfo(int id, double sideLength, double x, double y, double z, double pitch, double roll, double yaw) {
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
    public AprilTagInfo(int id, double sideLength, double x, double y, double z) {
        this(id, sideLength, x, y, z, 0, 0, 0);
    }
}
