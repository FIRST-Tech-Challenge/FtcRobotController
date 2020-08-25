package org.firstinspires.ftc.teamcode.rework.Tests;

import org.firstinspires.ftc.teamcode.rework.AutoTools.Point;

import static org.firstinspires.ftc.teamcode.rework.AutoTools.MathFunctions.*;

public class MathFunctionsTest {

    static void assertTrue(boolean condition) {
        if (!condition)
            throw new AssertionError();
    }

    static void testAngleWrap() {
        assertTrue(angleWrap(Math.PI * 7.4) == 1.4 * Math.PI);
    }

    static void testLinePointDistance() {
        assertTrue(linePointDistance(new Point(1, 1), new Point(2, 2), new Point(4, 3)) == 0.4472135954999579);
    }

    static void testClosestPointOnLineToPoint() {
        Point point = closestPointOnLineToPoint(new Point(64, 23), new Point(41, 64), new Point(98,12));
        assertTrue(point.x == 73.96673945909627);
        assertTrue(point.y == 33.92507979170165);
    }

    static void testTwoLineIntersectionPoint() {
        Point point = twoLineIntersectionPoint(new Point(-2, 6), 69, new Point(4,20), 420);
        assertTrue(point.x == 5.139601139601139);
        assertTrue(point.y == 498.63247863247864);
    }

    static void testSolveQuadratic() {
        double[] roots = solveQuadratic(1, 1, -2);
        assertTrue(roots[0] == 1);
        assertTrue(roots[1] == -2);
    }

    static void testLineCircleIntersection() {
        assertTrue(lineCircleIntersection(new Point(-1,-1), 7, new Point(3,6), new Point(-3,-6))
                .equals(new Point(2.5240998703626616, 5.048199740725323)));
    }

    public static void main(String[] argv) {
        testLinePointDistance();
        testAngleWrap();
        testTwoLineIntersectionPoint();
        testSolveQuadratic();
        testLineCircleIntersection();
    }
}
