package org.firstinspires.ftc.teamcode.vision;

public class SkystoneTargetInfo {
    StonePos quarryPosition;
    double confidence;
    double centroidX;
    double centroidY;
    double centroidDepth;
    double width;
    double height;
    double errorX;
    double errorY;
    boolean finished = false;

    public SkystoneTargetInfo() {
        quarryPosition = StonePos.NONE_FOUND;
    }

    public SkystoneTargetInfo(double centroidX, double centroidY, double width, double height, StonePos quarryPosition){
        this.centroidX = centroidX;
        this.centroidY = centroidY;
        this.width = width;
        this.height = height;
        this.quarryPosition = quarryPosition;
    }

    public String toString() {
        return String.format("x: %.2f, y: %.2f, width: %.2f, height: %.2f, quarryPosition: %s", centroidX, centroidY, width, height, quarryPosition);
    }

    public StonePos getQuarryPosition() {
        return quarryPosition;
    }

    public void setQuarryPosition(StonePos quarryPosition) {
        this.quarryPosition = quarryPosition;
    }
}
