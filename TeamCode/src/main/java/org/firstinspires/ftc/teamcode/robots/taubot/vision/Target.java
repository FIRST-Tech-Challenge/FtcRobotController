package org.firstinspires.ftc.teamcode.robots.taubot.vision;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.opencv.core.RotatedRect;

public class Target {
    long timeStamp; //the timestamp on which the target was detected - nanos from the time streaming was started
    int targetNumber; //the order of the target in the frame
    Vector2d centroid; //this is the position of the centroid of the target in the frame it was detected
    RotatedRect fittedRect; //holds the bounding rect of the fitted ellipse
    double aspectRatio; //hold the aspect ratio of the fitted ellipse
    double areaPixels; //this is the area in pixels of the target
    double heightPixels; //the height of the ellipse fitted to the blob in the frame - subtract half of this value from the centroid to get the base position of the target
    double widthPixels; //the wide of the ellipse fitted to the target blob
    double orientation; //the angular orientation of the blob in the frame
    boolean upright; //an assessment of the vertical orientation of the can
    double cameraHeading; //this is the heading of the target in the camera's fov when the target was last detected. it is only relevant for the frame where the target was detected
    Vector2d fieldPosition; //this is the estimated position of the target/can on the field
    boolean isCan; //do we think this is a can?

    public long getTimeStamp() {
        return timeStamp;
    }

    public void setTimeStamp(long timeStamp) {
        this.timeStamp = timeStamp;
    }

    public int getTargetNumber() {
        return targetNumber;
    }

    public void setTargetNumber(int targetNumber) {
        this.targetNumber = targetNumber;
    }

    public Vector2d getCentroid() {
        return centroid;
    }

    public void setCentroid(Vector2d centroid) {
        this.centroid = centroid;
    }

    public RotatedRect getFittedRect() {
        return fittedRect;
    }

    public void setFittedRect(RotatedRect fittedRect) {
        this.fittedRect = fittedRect;
    }

    public double getAspectRatio() {
        return aspectRatio;
    }

    public void setAspectRatio(double aspectRatio) {
        this.aspectRatio = aspectRatio;
    }

    public double getAreaPixels() {
        return areaPixels;
    }

    public void setAreaPixels(double areaPixels) {
        this.areaPixels = areaPixels;
    }

    public double getHeightPixels() {
        return heightPixels;
    }

    public void setHeightPixels(double heightPixels) {
        this.heightPixels = heightPixels;
    }

    public double getWidthPixels() {
        return widthPixels;
    }

    public void setWidthPixels(double widthPixels) {
        this.widthPixels = widthPixels;
    }

    public double getOrientation() {
        return orientation;
    }

    public void setOrientation(double orientation) {
        this.orientation = orientation;
    }

    public boolean isUpright() {
        return upright;
    }

    public void setUpright(boolean upright) {
        this.upright = upright;
    }

    public double getCameraHeading() {
        return cameraHeading;
    }

    public void setCameraHeading(double cameraHeading) {
        this.cameraHeading = cameraHeading;
    }

    public Vector2d getFieldPosition() {
        return fieldPosition;
    }

    public void setFieldPosition(Vector2d fieldPosition) {
        this.fieldPosition = fieldPosition;
    }

    public boolean isCan() {
        return isCan;
    }

    public void setCan(boolean can) {
        isCan = can;
    }

    //constructors
    public Target(){}
    public Target(long timeStamp, int targetNumber) {
        this.timeStamp = timeStamp;
        this.targetNumber = targetNumber;
    }
    public Target(long timeStamp, int targetNumber, Vector2d centroid, double cameraHeading) {
        this.timeStamp = timeStamp;
        this.targetNumber = targetNumber;
        this.centroid = centroid;
        this.cameraHeading = cameraHeading;
    }

    public double HeadingTo(Vector2d location){
        return this.getFieldPosition().angleBetween(location);
    }
    public double HeadingFrom(Vector2d location){
        return location.angleBetween(this.getFieldPosition());
    }
    public double Distance(Vector2d location){
        return location.distTo(this.getFieldPosition());
    }
}
