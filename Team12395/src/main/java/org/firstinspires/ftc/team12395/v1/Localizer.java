package org.firstinspires.ftc.team12395.v1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * Interface for localization methods.
 */
public interface Localizer {
    void setPose(Pose2d pose);

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     * @return the Localizer's current pose
     */
    Pose2d getPose();

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    PoseVelocity2d update();
}
