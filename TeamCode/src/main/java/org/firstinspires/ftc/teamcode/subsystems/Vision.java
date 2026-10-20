package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.objects.Subsystem;

/**
 * Something that sees AprilTags.
 *
 * <p>Every loop, {@link #Process()} looks at the newest picture and remembers the nearest
 * tag. Aim assist then asks three questions: is there a tag, which way is it, and how far.
 * Two cameras can answer them ({@link Vision_Limelight}, {@link Vision_Webcam}), and
 * {@link Vision_None} answers "no" when the robot has no camera. The drive never learns
 * which one it was handed. See docs/decisions/0011-vision-aim-assist.md.
 */
public interface Vision extends Subsystem {
    /** @return true when a tag was seen recently enough to trust */
    boolean SeesTag();

    /** @return the id of the nearest tag, or 0 when none is seen */
    int GetTagId();

    /**
     * @return degrees from straight ahead to the nearest tag. Left is positive, like yaw.
     *     0 when no tag is seen.
     */
    double GetBearing();

    /** @return inches from the camera to the nearest tag, or 0 when the camera can't tell */
    double GetRange();

    /** @return how many tags are in view right now */
    int CountTags();
}
