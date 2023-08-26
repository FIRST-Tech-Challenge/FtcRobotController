package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers;

public class LocalizerFactory {
    public static Tracker getTracker(Tracker.TrackType localizerType){
        if(localizerType == Tracker.TrackType.ODOMETRY){
            return new OdometryTracker();
        }
        else if(localizerType == Tracker.TrackType.ODOMETRY_IMU_LEFT){
            return new OdometryIMUTracker(false);
        }
        else if(localizerType == Tracker.TrackType.ODOMETRY_IMU_RIGHT){
            return new OdometryIMUTracker(true);
        }
        return null;
    }
}