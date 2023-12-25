package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import java.util.function.DoubleSupplier;

public class PoseEstimator extends HolonomicOdometry {

    public PoseEstimator(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder, DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset) {
        super(leftEncoder, rightEncoder, horizontalEncoder, trackWidth, centerWheelOffset);
    }

    public PoseEstimator(Pose2d initialPose, double trackwidth, double centerWheelOffset) {
        super(initialPose, trackwidth, centerWheelOffset);
    }

    public PoseEstimator(double trackwidth, double centerWheelOffset) {
        super(trackwidth, centerWheelOffset);
    }

    public void cameraMeasurements(Pose2d newPosition){
        robotPose = newPosition;
    }
}
