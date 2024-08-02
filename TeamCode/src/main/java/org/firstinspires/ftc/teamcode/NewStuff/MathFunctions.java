package org.firstinspires.ftc.teamcode.NewStuff;

import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.NewStuff.Navigation.Point;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class MathFunctions {
    public static double angleWrapRad(double angle) {
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static Point add(Point p1, Point p2) {
        return new Point(
                p1.getX() + p2.getX(),
                p1.getY() + p2.getY()
        );
    }
    public static Point scale(Point vector, double factor) {
        return new Point(
                vector.getX() * factor,
                vector.getY() * factor
        );
    }
    public static double dot(Point point1, Point point2) {
        return (point1.getX() * point2.getX()) + (point1.getY() * point2.getY());
    }
    public static double norm(Point vector) {
        return Math.sqrt(dot(vector, vector));
    }
    public static double distance(Point p1, Point p2) {
        return norm(add(p1, scale(p2, -1)));
    }
    public static Point normalize(Point vector){
        return scale(vector, 1 / norm(vector));
    }
    public static Point project(Point point, Point vector) {
        return scale(vector, dot(point, vector)/dot(vector, vector));
    }
    public static Point lineCircleIntersection(Point start, Point finish, Point current, double radius) throws ArithmeticException {
        Point vector = add(finish, scale(start, -1));
        Point shifted = lineCircleIntersection(vector, current, radius);
        return add(shifted, start);
    }
    private static Point lineCircleIntersection(Point vector, Point current, double radius) throws ArithmeticException {
        Point projection = project(current, vector);
        double distance = distance(current, projection);

        Point follow;
        if (distance > radius) {
            // edge case, handle no intersection
            throw new ArithmeticException("no intersection");
        } else {
            follow = add(projection,
                    scale(
                            normalize(vector),
                            Math.sqrt(Math.pow(radius, 2) - Math.pow(distance, 2))
                    )
            );
        }
        // edge case, handle ||f|| > ||v|| (past the point)
        if (norm(follow) > norm(vector)) follow = vector;
        // edge case, handle f behind v, then no intersection
        if (dot(follow, vector) < 0) throw new ArithmeticException("no intersection");
        return follow;
    }
}
