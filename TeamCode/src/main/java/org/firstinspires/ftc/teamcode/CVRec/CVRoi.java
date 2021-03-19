package org.firstinspires.ftc.teamcode.CVRec;

import org.opencv.core.Mat;

import java.util.Objects;

public class CVRoi implements Comparable<CVRoi> {
    private Mat input;
    private double distance = 0;
    private double angle = 0;
    private boolean clockwise = false;
    private int index = -1;

    public Mat getInput() {
        return input;
    }

    public void setInput(Mat input) {
        this.input = input;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public boolean isClockwise() {
        return clockwise;
    }

    public void setClockwise(boolean clockwise) {
        this.clockwise = clockwise;
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    @Override
    public int compareTo(CVRoi cvRoi) {
        return Integer.compare(this.getIndex(), cvRoi.getIndex());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;

        return Objects.equals(getIndex(), ((CVRoi) o).getIndex());
    }

    @Override
    public int hashCode() {
        return Objects.hash(getIndex());
    }
}
