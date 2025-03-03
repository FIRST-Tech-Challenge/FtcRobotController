package org.firstinspires.ftc.teamcode.Features;

import com.acmerobotics.roadrunner.Pose2d;

public class BuilderFunctions {

    // TODO: Use this as an base for a way point so that its faster and easier
    /*

        name = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[] {
                x * Tile, y * Tile, theta
            }, BuilderFunctions.RobotSides.FRONT
            ), WayPoint.WaypointType.DEFAULT
        ).max_radius(v).build()

    */

    public static double /*-inches-*/
        robotY = 15,
        robotX = 15,
        Tile = 24;

    public enum RobotSides {FRONT, REAR, CENTER, LEFT, RIGHT}

    /**
     * Adjusts the target pose in order for a certain side of the robot to meet the target point
     * @param pose
     * @param side
     * @return
     */
    public static double[] poseAdjuster(double[] pose, RobotSides side) {
        if (side == RobotSides.CENTER)
            return pose;

        double X = pose[0], Y = pose[1], H = pose[2],
            Afb = (robotY / 2) * Math.sin(H), Bfb = (robotY / 2) * Math.cos(H),
            Arl = (robotX / 2) * Math.sin(H), Brl = (robotX / 2) * Math.cos(H);

        switch (side) {
            case FRONT:
                X -= Bfb;
                Y -= Afb;
                break;
            case REAR:
                X += Bfb;
                Y += Afb;
                break;
            case LEFT:
                X += Arl;
                Y -= Brl;
                break;
            default: // RIGHT
                X -= Arl;
                Y += Brl;
                break;
        }

        return new double[]{X, Y, H};
    }

    public static Pose2d poseAdjuster(Pose2d pose, RobotSides side) {
        if (side == RobotSides.CENTER)
            return pose;

        double X = pose.position.x, Y = pose.position.y, H = pose.heading.toDouble(),
            Afb = (robotY / 2) * Math.sin(H), Bfb = (robotY / 2) * Math.cos(H),
            Arl = (robotX / 2) * Math.sin(H), Brl = (robotX / 2) * Math.cos(H);

        switch (side) {
            case FRONT:
                X -= Bfb;
                Y -= Afb;
                break;
            case REAR:
                X += Bfb;
                Y += Afb;
                break;
            case LEFT:
                X += Arl;
                Y -= Brl;
                break;
            default: // RIGHT
                X -= Arl;
                Y += Brl;
                break;
        }

        return new Pose2d(X, Y, H);
    }
}