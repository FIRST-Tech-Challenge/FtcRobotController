package org.firstinspires.ftc.teamcode.maps;

public class AprilTag {
    String name;
    int height;
    int x;
    int y;
    int[] detectionArea;

    public void setAttributes(String namePassed, int heightPassed, int xPassed, int yPassed, int[] detectionAreaPassed) {
        name = namePassed;
        height = heightPassed;
        x = xPassed;
        y = yPassed;
        detectionArea = detectionAreaPassed;
    }

    public String getName() {
        return name;
    }

    public int getHeight() {
        return height;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public boolean getDetectionArea() {
        return true;
    }
}