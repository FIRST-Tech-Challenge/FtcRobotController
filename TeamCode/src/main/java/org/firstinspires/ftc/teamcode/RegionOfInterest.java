package org.firstinspires.ftc.teamcode;

public class RegionOfInterest {
    public int leftBound;
    public int rightBound;
    public int count;

    public RegionOfInterest() {
        leftBound = 0;
        rightBound = 0;
        count = 0;
    }

    public RegionOfInterest(int ROILeftBound, int ROIRightBound) {
        leftBound = ROILeftBound;
        rightBound = ROIRightBound;
        count = 0;
    }
}
