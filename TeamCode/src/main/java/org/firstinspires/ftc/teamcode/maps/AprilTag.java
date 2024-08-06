package org.firstinspires.ftc.teamcode.maps;

public class AprilTag {
    private String description;
    private int x, y, z;
    private int[] detectionArea;

    public AprilTag(String description, int x, int y, int z, int[] detectionArea) {
        this.description = description;
        this.x = x;
        this.y = y;
        this.z = z;
        this.detectionArea = detectionArea;
    }
    @Deprecated
    public void setAttributes(String description, int x, int y, int z, int[] additionalAttributes) {
        this.description = description;
        this.x = x;
        this.y = y;
        this.z = z;
        this.detectionArea = additionalAttributes;
    }

    @Override
    public String toString() {
        return "AprilTag{" +
                "description='" + description + '\'' +
                ", x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }

    // Getters
    public String getDescription() {
        return description;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getZ() {
        return z;
    }

    public int[] getDetectionArea() {
        return detectionArea;
    }
}
