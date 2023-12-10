package org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
    Utility for - you've guessed it! - storing a AprilTagInfo object together with a AprilTagDetection object.
*/

public class InfoWithDetection {
    public AprilTagInfo info;
    public AprilTagDetection detection;

    public InfoWithDetection(AprilTagInfo info, AprilTagDetection detection) {
        this.info = info;
        this.detection = detection;
    }
}
