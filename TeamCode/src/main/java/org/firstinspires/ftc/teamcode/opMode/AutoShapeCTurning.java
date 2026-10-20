package org.firstinspires.ftc.teamcode.opMode;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import java.util.List;

/**
 * Drives a C, turning like a car: forward, around to the left, and back toward the
 * driver. It ends two tiles to the left, facing the driver.
 *
 * <pre>
 *    .----.
 *   /      \
 *  |        |
 *  v        R
 * </pre>
 *
 * <p>Pictures are from the driver's view, standing behind the robot. "R" is where the robot
 * starts, facing up the page. How Pedro measures is explained in {@link PedroAuto}.
 */
@Autonomous(name = "Shape: C (turning)", group = "Shapes")
public class AutoShapeCTurning extends PedroAuto {
    /** How big the shape is, in inches. One field tile is 24 inches, so 48 is two tiles. */
    private static final double shapeSize = 48;

    @Override
    protected List<Path> BuildPaths() {
        // A Bezier curve starts at the first point and ends at the last. The middle two are
        // "pull" points: the curve bends toward them but never touches them.
        Pose start = StartPose();
        Pose pullForward = new Pose(shapeSize, 0);
        Pose pullForwardAndLeft = new Pose(shapeSize, shapeSize);
        Pose end = new Pose(0, shapeSize);

        // .tangent(): face the way the curve is going, like a car.
        Path cCurve = curve(start, pullForward, pullForwardAndLeft, end).tangent();

        List<Path> paths = new ArrayList<>();
        paths.add(cCurve);
        return paths;
    }
}
