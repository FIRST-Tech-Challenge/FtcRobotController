package org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers;

import com.arcrobotics.ftclib.kinematics.Odometry;

public class LocalizerFactory {
    public static Tracker getChassis(Tracker.TrackType localizerType){
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