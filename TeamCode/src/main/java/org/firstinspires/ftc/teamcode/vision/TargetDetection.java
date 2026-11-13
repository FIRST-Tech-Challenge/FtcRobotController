package org.firstinspires.ftc.teamcode.vision;

/**
 * One detected target in image coordinates.
 * This result does not contain a confidence score: color thresholds do not
 * produce the calibrated probabilities associated with trained ML models.
 */
public final class TargetDetection {
    private final String label;
    private final double centerXPx;
    private final double centerYPx;
    private final int areaPx;
    private final double horizontalError;

    public TargetDetection(String label, double centerXPx, double centerYPx,
                           int areaPx, int frameWidthPx) {
        if (frameWidthPx <= 0) {
            throw new IllegalArgumentException("Frame width must be positive");
        }
        this.label = label;
        this.centerXPx = centerXPx;
        this.centerYPx = centerYPx;
        this.areaPx = areaPx;
        // -1 = left edge, 0 = image center, +1 = right edge.
        this.horizontalError = (2.0 * centerXPx / frameWidthPx) - 1.0;
    }

    public String getLabel() {
        return label;
    }

    public double getCenterXPx() {
        return centerXPx;
    }

    public double getCenterYPx() {
        return centerYPx;
    }

    public int getAreaPx() {
        return areaPx;
    }

    public double getHorizontalError() {
        return horizontalError;
    }
}
