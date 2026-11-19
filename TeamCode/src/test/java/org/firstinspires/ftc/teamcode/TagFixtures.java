package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/**
 * Builds a physically consistent metric tag pose for unit tests: the sticker lies in the measured
 * 30-deg underside plane, its +X edge runs horizontally along the cell (far -X, audience +X), and its
 * outward +Z normal faces down toward the robot side. Orientation is expressed in the robot's
 * intrinsic Z-Y-X tag basis (after LimelightObservation's conversion), so heading can be read from
 * the edge exactly as on the robot.
 */
final class TagFixtures {
    private TagFixtures() { }

    static TagObservation seen(GoalTarget target, int id, MotionSample robot, double turretYaw, ShooterMount mount) {
        GoalTarget.Tag known = target.tags.get(id);
        double cameraYaw = robot.heading + mount.zeroYaw + turretYaw + mount.cameraYaw;
        Vec ray = new Vec(known.x, known.y).minus(mount.cameraPosition(robot, turretYaw));
        double[] position = toCamera(ray.x, ray.y, known.height - mount.cameraHeight, cameraYaw, mount.cameraPitch);
        // Upright sticker seen from the robot: +X camera-right, outward normal toward the robot side.
        double edgeYaw = PoseHistory.tagEdgeFieldYaw(target), normalYaw = edgeYaw - Math.PI / 2;
        double plane = Math.toRadians(HiveTagGeometry.RAISED_TAG_PLANE_DEG);
        double[] x = toCamera(Math.cos(edgeYaw), Math.sin(edgeYaw), 0, cameraYaw, mount.cameraPitch);
        double[] z = toCamera(Math.cos(normalYaw) * Math.sin(plane), Math.sin(normalYaw) * Math.sin(plane),
                -Math.cos(plane), cameraYaw, mount.cameraPitch);
        double[] y = {z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]};
        // R = Rz(yaw) Ry(pitch) Rx(roll), columns are the tag axes in camera coordinates.
        double pitch = Math.asin(-x[2]), roll = Math.atan2(y[2], z[2]), yaw = Math.atan2(x[1], x[0]);
        return new TagObservation(id, position[0], position[1], position[2], roll, pitch, yaw, 1);
    }

    /** Field (east, north, up) vector to optical camera (right, down, forward). Inverse of TagObservation.level. */
    static double[] toCamera(double east, double north, double up, double cameraYaw, double cameraPitch) {
        Vec horizontal = new Vec(east, north).rotate(-cameraYaw);
        double forward = horizontal.x, left = horizontal.y;
        return new double[]{-left, forward * Math.sin(cameraPitch) - up * Math.cos(cameraPitch),
                forward * Math.cos(cameraPitch) + up * Math.sin(cameraPitch)};
    }
}
