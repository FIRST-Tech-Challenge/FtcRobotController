package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Field Constants
 * Where our robot STARTS and PARKS in Autonomous, written in Pedro's coordinates.
 * Two Autos need these numbers (AutoLeaveAndPark and ShootAndParkAuto), so they live here once.
 * (ARCHITECTURE.md: a shared constants class is made when a second class needs the value.)
 *
 * PEDRO'S MAP OF THE BIOBUZZ FIELD
 *   Picture the field the way the Competition Manual draws it, with the AUDIENCE at the BOTTOM,
 *   the RED alliance area on the LEFT and the BLUE alliance area on the RIGHT.
 *     - The ORIGIN (0, 0) is the bottom-left inside corner of the field (the corner of tile A1).
 *     - X grows to the RIGHT (toward the blue side). Y grows UP (toward the top wall).
 *     - Heading 0 degrees faces +X (right). Turning LEFT (counterclockwise) is a positive heading.
 *       So facing UP the field is +90, facing LEFT is 180, and facing DOWN is -90.
 *     - The field is 141.5 inches square (the picture TeamCode/docs/pedro-fieldcoordinates-dark.png shows
 *       Pedro's corners (0, 0) to (141.5, 141.5) and the middle at (70.75, 70.75)). Each tile is
 *       about 23.6 inches. Column D is X about 70.75 to 94.3. Row 6 is Y about 117.9 to 141.5.
 *
 * OUR BLUE PLAN (the numbers below)
 *   START  : tile D6, touching the TOP wall, facing DOWN (-90 degrees) toward the hive. The intake
 *            (front of the robot) and the turret both point at the hive, so the camera can lock
 *            on right away.
 *   PARK   : our LOADING ZONE, tile F2 on the RIGHT wall. The zone is about 11 inches deep (from
 *            the wall) and 23 inches wide (Y about 23.6 to 47.2).
 *   WAYPOINT: a stop on the way, to the right of the hive, so the robot does not clip the hive.
 *
 * RED IS A ROTATION, NOT A MIRROR
 *   Red's LOADING ZONE is at the top-LEFT and blue's is at the bottom-RIGHT, so the field is the
 *   same after turning it 180 degrees around its middle. It is NOT the same flipped left-to-right.
 *   So every red pose is the blue pose rotated 180 degrees:
 *       red X = FIELD_SIZE - blue X,   red Y = FIELD_SIZE - blue Y,   red heading = blue + 180.
 *   Blue D6 (top wall, facing down) becomes red C1 (bottom wall, facing up).
 *   We only write the BLUE numbers. The red ones are built from them in makePose(), so there are
 *   never two sets of numbers that can disagree.
 *
 * WHY THE PARK SPOT IS NOT TOUCHING THE WALL
 *   LEAVE (3 points) needs the robot to be NOT touching the perimeter wall when AUTO ends.
 *   PARK (5 points) needs the robot to be AT LEAST PARTLY inside the LOADING ZONE.
 *   So we aim the robot's edge halfway into the zone: about 5.5 inches off the wall and 5.5 inches
 *   inside the zone line. Odometry drifts a few inches, and this way a small miss in either
 *   direction still scores both.
 *
 * TODO (issue #55): The field size comes from Pedro's field picture, and the other numbers are estimates
 *   read from the field pictures. Before trusting them at an event:
 *   - check the real inside width of the field with a tape measure (FIELD_SIZE_INCHES, 141.5),
 *   - measure the robot (ROBOT_WIDTH / LENGTH) and where its center of turning is,
 *   - check the loading zone's real depth,
 *   - drive the Auto and look at where the robot really ends up.
 *   A GitHub issue tracks the measurements.
 */
public class FieldConstants {

    // ------------------------------------------------------------------
    // The field
    // ------------------------------------------------------------------
    // The inside width and length of the field, in inches: 141.5, from Pedro's field picture
    // (TeamCode/docs/pedro-fieldcoordinates-dark.png). It is a little under six 24 inch tiles (144).
    // TODO (issue #55): check it wall to wall on the real field. This matters for the RED rotation and for
    // the parking spot next to the far wall.
    public static final double FIELD_SIZE_INCHES = 141.5;

    // ------------------------------------------------------------------
    // The robot
    // ------------------------------------------------------------------
    // Width is side to side. Length is front to back. TODO (issue #55): These are the coach's numbers;
    // a GitHub issue asks for them to be re-measured, and for the robot's center of turning
    // (the point Pedro and the Pinpoint call "the robot") to be checked. We assume the center of
    // turning is the middle of this rectangle.
    public static final double ROBOT_WIDTH_INCHES = 16.5;
    public static final double ROBOT_LENGTH_INCHES = 17.75;

    // ------------------------------------------------------------------
    // The loading zone (blue: tile F2 on the right wall)
    // ------------------------------------------------------------------
    // How deep the zone is, measured out from the wall.
    public static final double LOADING_ZONE_DEPTH_INCHES = 11.0;

    // Where the middle of the zone is along the wall. It is tile row 2, which covers Y about 23.6
    // to 47.2 (read from Pedro's field picture), so the middle is 35.4.
    public static final double LOADING_ZONE_CENTER_Y_INCHES = 35.4;

    // ------------------------------------------------------------------
    // BLUE poses. All three use the same heading, so the robot never turns during the Auto.
    // It faces down the field and slides (strafes) sideways where it needs to go right.
    // ------------------------------------------------------------------
    // -90 and 270 are the same direction (facing down). We use -90 because Pedro and the
    // Pinpoint keep headings between -180 and 180, so there is no "wrap around" to worry about.
    public static final double BLUE_HEADING_DEGREES = -90.0;

    // START: the middle of tile D (3.5 tiles from the left wall), with the robot's back against
    // the top wall. The center of the robot is half its length away from the wall.
    // Place the robot by measuring from the LEFT wall to the robot's center: about 82.5 inches
    // (3.5 tiles of 141.5 / 6 = 23.6 inches each).
    public static final double BLUE_START_X_INCHES = 3.5 * FIELD_SIZE_INCHES / 6.0;
    public static final double BLUE_START_Y_INCHES = FIELD_SIZE_INCHES - ROBOT_LENGTH_INCHES / 2.0;

    // WAYPOINT: to the right of the hive, with room to spare on both sides. The hive's right
    // edge is at about X 94, and the wall is at 141.5, so X 118 keeps the robot (8.25 inches
    // each side of its center) about 15 inches from the hive and about 15 inches from the wall,
    // well away from the flower on the right wall (near Y 95).
    public static final double BLUE_WAYPOINT_X_INCHES = 118.0;
    public static final double BLUE_WAYPOINT_Y_INCHES = 110.0;

    // PARK: the robot's right edge halfway into the zone (see above). The robot's center is half
    // its width left of that edge.
    public static final double BLUE_PARK_X_INCHES = FIELD_SIZE_INCHES
            - LOADING_ZONE_DEPTH_INCHES / 2.0
            - ROBOT_WIDTH_INCHES / 2.0;
    public static final double BLUE_PARK_Y_INCHES = LOADING_ZONE_CENTER_Y_INCHES;

    // A PoseFactory makes poses (an X, a Y and a heading) from numbers in DEGREES.
    private static final PoseFactory POSE_FACTORY = PoseFactory.degrees();

    // ------------------------------------------------------------------
    // The poses an Auto asks for. Pass the alliance and get the right pose back.
    // ------------------------------------------------------------------
    public static Pose startPose(VisionSubsystem.Alliance alliance) {
        return makePose(alliance, BLUE_START_X_INCHES, BLUE_START_Y_INCHES, BLUE_HEADING_DEGREES);
    }

    public static Pose waypointPose(VisionSubsystem.Alliance alliance) {
        return makePose(alliance, BLUE_WAYPOINT_X_INCHES, BLUE_WAYPOINT_Y_INCHES, BLUE_HEADING_DEGREES);
    }

    public static Pose parkPose(VisionSubsystem.Alliance alliance) {
        return makePose(alliance, BLUE_PARK_X_INCHES, BLUE_PARK_Y_INCHES, BLUE_HEADING_DEGREES);
    }

    /*
     * Builds a pose from BLUE numbers. For BLUE it uses them as they are. For RED it turns the
     * pose 180 degrees around the middle of the field (see "RED IS A ROTATION" above).
     */
    private static Pose makePose(VisionSubsystem.Alliance alliance,
                                 double blueX, double blueY, double blueHeadingDegrees) {
        double x = blueX;
        double y = blueY;
        double headingDegrees = blueHeadingDegrees;

        if (alliance == VisionSubsystem.Alliance.RED) {
            x = FIELD_SIZE_INCHES - blueX;
            y = FIELD_SIZE_INCHES - blueY;
            headingDegrees = wrapDegrees(blueHeadingDegrees + 180.0);
        }

        return POSE_FACTORY.of(x, y, headingDegrees);
    }

    // Keeps an angle between -180 and 180 degrees. (-90 + 180 = 90 is fine, but 90 + 180 = 270
    // would become -90.)
    private static double wrapDegrees(double degrees) {
        double wrapped = degrees;
        while (wrapped > 180.0) {
            wrapped = wrapped - 360.0;
        }
        while (wrapped <= -180.0) {
            wrapped = wrapped + 360.0;
        }
        return wrapped;
    }
}
