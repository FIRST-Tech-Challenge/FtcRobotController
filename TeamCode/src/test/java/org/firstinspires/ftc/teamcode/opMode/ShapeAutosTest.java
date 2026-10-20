package org.firstinspires.ftc.teamcode.opMode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Checks each shape auto goes where its picture says, without a robot. {@code path.get(t)}
 * asks Pedro "where should the robot be at this point in the path?" t = 0 is the start,
 * 0.5 halfway, 1 the end. Pedro's x is forward, y is left, heading in radians (left turn
 * is positive). The shapes are 48 inches, two tiles.
 */
@DisplayName("Shape autos")
class ShapeAutosTest {
    private static final double closeEnough = 1e-6;
    private static final double size = 48;

    private static void assertPose(double x, double y, double headingDegrees, Pose actual) {
        assertEquals(x, actual.x(), closeEnough, "x (forward) in " + actual);
        assertEquals(y, actual.y(), closeEnough, "y (left) in " + actual);
        assertEquals(Math.toRadians(headingDegrees), actual.heading(), closeEnough, "heading in " + actual);
    }

    @Nested
    @DisplayName("Shape: C (turning)")
    class CTurning {
        private final List<Path> paths = new AutoShapeCTurning().BuildPaths();

        @Test
        @DisplayName("is one curve that starts at the start spot, facing forward")
        void start() {
            assertEquals(1, paths.size());
            assertPose(0, 0, 0, paths.get(0).get(0));
        }

        @Test
        @DisplayName("halfway: 3/4 of the way out, halfway left, facing left")
        void middle() {
            // A cubic Bezier at t = 0.5 is (P0 + 3*P1 + 3*P2 + P3) / 8. For x: (0 + 3*48 + 3*48 + 0) / 8 = 36.
            assertPose(0.75 * size, 0.5 * size, 90, paths.get(0).get(0.5));
        }

        @Test
        @DisplayName("ends two tiles to the left, facing back toward the driver")
        void end() {
            assertPose(0, size, 180, paths.get(0).endPose());
        }
    }

    @Nested
    @DisplayName("Shape: C (no turning)")
    class CNoTurning {
        private final List<Path> paths = new AutoShapeCNoTurning().BuildPaths();

        @Test
        @DisplayName("follows the same curve as the turning C")
        void sameCurve() {
            Path turning = new AutoShapeCTurning().BuildPaths().get(0);
            for (double t = 0; t <= 1.0; t += 0.125) {
                Pose withTurn = turning.get(t);
                Pose withoutTurn = paths.get(0).get(t);
                assertEquals(withTurn.x(), withoutTurn.x(), closeEnough);
                assertEquals(withTurn.y(), withoutTurn.y(), closeEnough);
            }
        }

        @Test
        @DisplayName("faces forward the whole way")
        void alwaysForward() {
            for (double t = 0; t <= 1.0; t += 0.125) {
                assertEquals(0, paths.get(0).get(t).heading(), closeEnough, "at t=" + t);
            }
        }
    }

    @Nested
    @DisplayName("Shape: Z")
    class ZShape {
        private final List<Path> paths = new AutoShapeZ().BuildPaths();

        @Test
        @DisplayName("is three straight lines: slide left, diagonal forward-right, slide left")
        void corners() {
            assertEquals(3, paths.size());
            assertPose(0, 0, 0, paths.get(0).get(0));
            assertPose(0, size, 0, paths.get(0).endPose());
            assertPose(size, 0, 0, paths.get(1).endPose());
            assertPose(size, size, 0, paths.get(2).endPose());
        }

        @Test
        @DisplayName("each line starts where the last one ended")
        void connected() {
            for (int i = 1; i < paths.size(); i++) {
                Pose previousEnd = paths.get(i - 1).endPose();
                Pose thisStart = paths.get(i).get(0);
                assertEquals(previousEnd.x(), thisStart.x(), closeEnough, "path " + (i + 1));
                assertEquals(previousEnd.y(), thisStart.y(), closeEnough, "path " + (i + 1));
            }
        }

        @Test
        @DisplayName("the diagonal crosses the middle of the Z")
        void diagonalMiddle() {
            assertPose(size / 2, size / 2, 0, paths.get(1).get(0.5));
        }

        @Test
        @DisplayName("faces forward the whole way")
        void alwaysForward() {
            for (Path path : paths) {
                for (double t = 0; t <= 1.0; t += 0.25) {
                    assertEquals(0, path.get(t).heading(), closeEnough);
                }
            }
        }
    }

    @Test
    @DisplayName("every auto starts at (0, 0) facing forward unless it says otherwise")
    void startPose() {
        assertPose(0, 0, 0, new AutoShapeZ().StartPose());
    }
}
