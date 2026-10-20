package org.firstinspires.ftc.teamcode.opMode;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import java.util.List;

/**
 * Drives the same C as "Shape: C (turning)" without turning. Mecanum wheels can slide
 * sideways, so the robot faces forward the whole time: forward, sliding left, backing up.
 *
 * <pre>
 *    .----.
 *   /      \
 *  |        |
 *  v        R
 * </pre>
 *
 * <p>How Pedro measures is explained in {@link PedroAuto}.
 */
@Autonomous(name = "Shape: C (no turning)", group = "Shapes")
public class AutoShapeCNoTurning extends PedroAuto {
    /** How big the shape is, in inches. One field tile is 24 inches, so 48 is two tiles. */
    private static final double shapeSize = 48;

    @Override
    protected List<Path> BuildPaths() {
        Pose start = StartPose();
        Pose pullForward = new Pose(shapeSize, 0);
        Pose pullForwardAndLeft = new Pose(shapeSize, shapeSize);
        Pose end = new Pose(0, shapeSize);

        // .constant(0): keep facing heading 0, straight ahead, the whole way.
        double faceForward = 0;
        Path cCurve = curve(start, pullForward, pullForwardAndLeft, end).constant(faceForward);

        List<Path> paths = new ArrayList<>();
        paths.add(cCurve);
        return paths;
    }
}
