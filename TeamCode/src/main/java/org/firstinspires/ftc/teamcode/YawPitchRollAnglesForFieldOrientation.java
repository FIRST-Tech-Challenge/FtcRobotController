package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * A simplified view of a robot's orientation, according to the Robot Coordinate System, relative to
 * the robot sitting on level ground facing straight ahead (which direction is straight ahead can
 * vary).
 * <p>
 * In the Robot Coordinate System, the X axis extends horizontally from your robot to the right,
 * parallel to the ground. The Y axis extends horizontally from your robot straight ahead, parallel
 * to the ground. The Z axis extends vertically from your robot, towards the ceiling.
 * <p>
 * The Robot Coordinate System is right-handed, which means that if you point the thumb of a typical
 * human right hand in the direction of an axis, rotation around that axis is defined as positive in
 * the direction that the fingers curl.
 * <p>
 * Yaw is the side-to-side lateral rotation of the robot. In terms of the Robot Coordinate
 * System, it is defined as how far the robot has turned around the Z axis. Sometimes yaw is also
 * referred to as "heading".
 * <p>
 * Pitch is the front-to-back rotation of the robot. In terms of the Robot Coordinate System, it is
 * how far the robot has turned around the X axis.
 * <p>
 * Roll is the side-to-side tilt of the robot. In terms of the Robot Coordinate System, it is
 * defined as how far the robot has turned around the Y axis.
 * <p>
 * All angles are in the range of -180 degrees to 180 degrees.
 * <p>
 * The angles are applied intrinsically, in the order of yaw, then pitch, then roll. "Intrinsically"
 * means that the axes move along with the object as you perform the rotations. As an example, if
 * the yaw is 30 degrees, the pitch is 40 degrees, and the roll is 10 degrees, that means that you
 * would reach the described orientation by first rotating a robot 30 degrees counter-clockwise
 * from the starting point, with all wheels continuing to touch the ground (rotation around the Z
 * axis). Then, you make your robot point 40 degrees upward (rotate it 40 degrees around the X
 * axis). Because the X axis moved with the robot, the pitch is not affected by the yaw value. Then
 * from that position, the robot is tilted 10 degrees to the right, around the newly positioned Y
 * axis, to produce the actual position of the robot.
 * <p>
 * Instances of this class should <i>only</i> be created with yaw, pitch, and roll angles that are
 * defined as described above. If you need to represent angles that are in a different format, use
 * the much more flexible {@link Orientation} class.
 */
public class YawPitchRollAnglesForFieldOrientation {
    private final double yawDegrees;
    private final double pitchDegrees;
    private final double rollDegrees;
    private final long acquisitionTime;

    /**
     * See the top-level class Javadoc for the format that these angles need to be in.
     */
    public YawPitchRollAnglesForFieldOrientation(AngleUnit angleUnit, double yaw, double pitch, double roll, long acquisitionTime) {
        this.yawDegrees = angleUnit.toDegrees(yaw);
        this.pitchDegrees = angleUnit.toDegrees(pitch);
        this.rollDegrees = angleUnit.toDegrees(roll);
        this.acquisitionTime = acquisitionTime;
    }

    /**
     * @param angleUnit The unit that will be used for the result.
     * @return The side-to-side lateral rotation of the robot (rotation around the Z axis),
     *         normalized to the range of [-180,+180) degrees.
     */
    public double getYaw(AngleUnit angleUnit) {
        return angleUnit.fromDegrees(yawDegrees);
    }



    /**
     * @param angleUnit The unit that will be used for the result.
     * @return The front-to-back rotation of the robot (rotation around the X axis), normalized to
     *         the range of [-180,+180) degrees
     */
    public double getPitch(AngleUnit angleUnit) {
        return angleUnit.fromDegrees(pitchDegrees);
    }

    /**
     * @param angleUnit The unit that will be used for the result.
     * @return The side-to-side tilt of the robot (rotation around the Y axis), normalized to
     *         the range of [-180,+180) degrees
     */
    public double getRoll(AngleUnit angleUnit) {
        return angleUnit.fromDegrees(rollDegrees);
    }

    /**
     * @return The time on the System.nanoTime() clock at which the data was acquired. If no
     * timestamp is associated with this particular set of data, this value is zero.
     */
    public long getAcquisitionTime() {
        return acquisitionTime;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "{yaw=%.3f, pitch=%.3f, roll=%.3f}", yawDegrees, pitchDegrees, rollDegrees);
    }
}
