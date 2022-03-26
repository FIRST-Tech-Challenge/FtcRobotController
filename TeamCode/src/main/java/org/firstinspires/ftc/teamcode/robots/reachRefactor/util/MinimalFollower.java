package org.firstinspires.ftc.teamcode.robots.reachRefactor.util;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MinimalFollower extends TrajectoryFollower {

    public MinimalFollower(PIDCoefficients axialCoeffs, PIDCoefficients headingCoeffs) {

    }

    @NonNull
    @Override
    public Pose2d getLastError() {
        return null;
    }

    @Override
    protected void setLastError(@NonNull Pose2d pose2d) {

    }

    @NonNull
    @Override
    protected DriveSignal internalUpdate(@NonNull Pose2d pose2d, @Nullable Pose2d pose2d1) {
        return null;
    }
}
