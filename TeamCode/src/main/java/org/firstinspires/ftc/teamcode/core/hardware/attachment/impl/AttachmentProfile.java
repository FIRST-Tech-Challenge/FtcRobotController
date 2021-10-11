package org.firstinspires.ftc.teamcode.core.hardware.attachment.impl;

public class AttachmentProfile {
    public final double maxPower;
    public final double minPower;

    public final double maxPositionMultiple;

    /**
     * @param maxPower the maximum speed of the attatchment
     * @param minPower the minimum speed of the attatchment
     * @param maxPositionMultiple the time to get to the max position at max speed.
     */
    public AttachmentProfile(double maxPower, double minPower, double maxPositionMultiple) {
        this.maxPower = maxPower;
        this.minPower = minPower;
        this.maxPositionMultiple = maxPositionMultiple;
    }
}
