package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc;


import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.angle;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis.EncoderChassis.ypos;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Arrays;
import java.util.List;

/**
 * Created by hyunC on 7/22/22.
 */

public class DrawingUtil {
    private static List<Vector2d> getFrontLeftWheelContour(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, -2),
                new Vector2d(3, -2),
                new Vector2d(3, 2),
                new Vector2d(-3, 2),
                new Vector2d(-3, -2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(angle).plus(new Vector2d(xpos, ypos)));
        }
        return wheelPath;
    }

    private static List<Vector2d> getFrontLeftWheelPattern(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, -2),
                new Vector2d(-1, 2),
                new Vector2d(1, 2),
                new Vector2d(-1, -2),
                new Vector2d(1, -2),
                new Vector2d(3, 2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.getHeading()).plus(wheelPose.vec()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getRearLeftWheelContour(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, 2),
                new Vector2d(3, 2),
                new Vector2d(3, -2),
                new Vector2d(-3, -2),
                new Vector2d(-3, 2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.getHeading()).plus(wheelPose.vec()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getRearLeftWheelPattern(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, 2),
                new Vector2d(-1, -2),
                new Vector2d(1, -2),
                new Vector2d(-1, 2),
                new Vector2d(1, 2),
                new Vector2d(3, -2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.getHeading()).plus(wheelPose.vec()));
        }
        return wheelPath;
    }

    public static void drawMecanumRobot(Canvas canvas, Pose2d robotPose) {
        canvas.setStrokeWidth(2);
        // robot body
        List<Vector2d> robotPath = Arrays.asList(
                new Vector2d(9, 9),
                new Vector2d(-9, 9),
                new Vector2d(-9, -9),
                new Vector2d(9, -9),
                new Vector2d(9, 0),
                new Vector2d(3, 0),
                new Vector2d(9, 0),
                new Vector2d(9, 9)
        );
        for (int i = 0; i < robotPath.size(); i++) {
            robotPath.set(i, robotPath.get(i).rotated(robotPose.getHeading()).plus(robotPose.vec()));
        }
        drawVectorPolyline(canvas, robotPath);

        // robot wheels
        List<Vector2d> wheelOrigins = Arrays.asList(
                new Vector2d(4.5, 5.5),
                new Vector2d(-4.5, 5.5),
                new Vector2d(-4.5, -5.5),
                new Vector2d(4.5, -5.5)
        );
        for (int i = 0; i < wheelOrigins.size(); i++) {
            Vector2d adjustedOrigin = wheelOrigins.get(i).rotated(robotPose.getHeading()).plus(robotPose.vec());
            if (i % 2 == 0) {
                drawVectorPolyline(canvas, getFrontLeftWheelContour(new Pose2d(adjustedOrigin, robotPose.getHeading())));
                drawVectorPolyline(canvas, getFrontLeftWheelPattern(new Pose2d(adjustedOrigin, robotPose.getHeading())));
            } else {
                drawVectorPolyline(canvas, getRearLeftWheelContour(new Pose2d(adjustedOrigin, robotPose.getHeading())));
                drawVectorPolyline(canvas, getRearLeftWheelPattern(new Pose2d(adjustedOrigin, robotPose.getHeading())));
            }
        }
    }

    private static void drawVectorPolyline(Canvas canvas, List<Vector2d> vectors) {
        double[] xCoords = new double[vectors.size()];
        double[] yCoords = new double[vectors.size()];
        for (int i = 0; i < xCoords.length; i++) {
            xCoords[i] = vectors.get(i).getX();
            yCoords[i] = vectors.get(i).getY();
        }
        canvas.strokePolyline(xCoords, yCoords);
    }
}