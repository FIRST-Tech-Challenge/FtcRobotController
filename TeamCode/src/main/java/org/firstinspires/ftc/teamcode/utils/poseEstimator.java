package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import java.util.function.DoubleSupplier;

public class poseEstimator extends HolonomicOdometry {
    public poseEstimator(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder, DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset) {
        super(leftEncoder, rightEncoder, horizontalEncoder, trackWidth, centerWheelOffset);
    }

    public poseEstimator(Pose2d initialPose, double trackwidth, double centerWheelOffset) {
        super(initialPose, trackwidth, centerWheelOffset);
    }

    public poseEstimator(double trackwidth, double centerWheelOffset) {
        super(trackwidth, centerWheelOffset);
    }
    public void cameraMeasurements(Rotation2d rotation2d, Translation2d translation2d){
        robotPose = new Pose2d(translation2d,rotation2d);
    }
}
