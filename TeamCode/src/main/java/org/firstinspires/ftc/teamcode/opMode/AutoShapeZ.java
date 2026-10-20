package org.firstinspires.ftc.teamcode.opMode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;
import java.util.List;

/**
 * Drives a Z with sharp corners, always facing forward: slide left, drive diagonally
 * forward and right, slide left. Only a mecanum robot can do this. (The robot draws the Z
 * starting from its bottom-right corner.)
 *
 * <pre>
 *  4 &lt;--- 3
 *        /
 *      /
 *    /
 *  2 &lt;--- R
 * </pre>
 *
 * <p>To make a new auto, copy this file: rename the class and the name in
 * {@code @Autonomous}, then change the points and lines in {@link #BuildPaths()}. How Pedro
 * measures is explained in {@link PedroAuto}.
 */
@Autonomous(name = "Shape: Z", group = "Shapes")
public class AutoShapeZ extends PedroAuto {
    /** How big the shape is, in inches. One field tile is 24 inches, so 48 is two tiles. */
    private static final double shapeSize = 48;

    @Override
    protected List<Path> BuildPaths() {
        double faceForward = 0;
        Pose corner1 = StartPose();
        Pose corner2 = new Pose(0, shapeSize);
        Pose corner3 = new Pose(shapeSize, 0);
        Pose corner4 = new Pose(shapeSize, shapeSize);

        // line(from, to): a straight path. .constant(0): face straight ahead the whole way.
        Path bottom = line(corner1, corner2).constant(faceForward);
        Path diagonal = line(corner2, corner3).constant(faceForward);
        Path top = line(corner3, corner4).constant(faceForward);

        List<Path> paths = new ArrayList<>();
        paths.add(bottom);
        paths.add(diagonal);
        paths.add(top);
        return paths;
    }
}
