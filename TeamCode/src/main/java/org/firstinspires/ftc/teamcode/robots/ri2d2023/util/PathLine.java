package org.firstinspires.ftc.teamcode.robots.ri2d2023.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/*class used for horizontal and vertical lines to act as paths
* */
public class PathLine {
    boolean horizontal;
    double initialVelocity;
    double maxVelocity;
    double acceleration;

    double t1;
    public double t2;
    double totalTime;


    double totalDistance;


    Pose2d start;
    double delta;

    double distanceToMaxVelocity;

    public PathLine(Pose2d startCoordinate, Pose2d endCoordinate, double velocity, double max_velocity, double a) {

        horizontal = startCoordinate.getY() == endCoordinate.getY();
        initialVelocity = velocity;
        maxVelocity = max_velocity;
        acceleration = a;
        totalDistance = horizontal ? Math.abs(startCoordinate.getX() - endCoordinate.getX()) : Math.abs(startCoordinate.getY() - endCoordinate.getY());
        start = startCoordinate;
        delta = horizontal ? Math.signum(endCoordinate.getX() - startCoordinate.getX()) : Math.signum(endCoordinate.getY() - startCoordinate.getY());

        t1 = (-2 * initialVelocity + Math.sqrt(4 * Math.pow(initialVelocity, 2) - 4 * acceleration * (Math.pow(initialVelocity, 2) / (2 * acceleration) - totalDistance))) / (2 * acceleration);
        totalTime = initialVelocity / acceleration + 2 * t1;
        t2 = t1;
        System.out.println(t1);
        if (t1 * acceleration + initialVelocity > maxVelocity) {

            t1 = (maxVelocity - initialVelocity) / acceleration;
            t2 = (totalDistance - initialVelocity * t1 - 0.5 * acceleration * Math.pow(t1, 2) - 0.5 * Math.pow(maxVelocity, 2) / acceleration) / maxVelocity + t1;
            totalTime = t2 + maxVelocity / acceleration;
        }
        distanceToMaxVelocity = initialVelocity * t1 + Math.pow(t1, 2) * acceleration / 2;
    }

    //constructor for path where initial velocity is the maximum velocity
    //used for a stopping path.
    //todo

    public Pose2d getPointAtTime(double time) {
        double distance = getDistanceAtTime(time);
        distance = Math.min(totalDistance, distance);
        System.out.println(delta);
        if (horizontal) {
            return new Pose2d(start.getX() + delta * distance, start.getY(), 0);
        }
        return new Pose2d(start.getX(), start.getY() + delta * distance, 0);
    }

    private double getDistanceAtTime(double time) {
        double result = 0;
        if (time >= totalTime) return totalDistance;
        if (time <= t1)
            return initialVelocity * time + 0.5 * acceleration * time * time;
        if (time <= t2) {

            return distanceToMaxVelocity + maxVelocity * (time - t1);
        }

        return totalDistance - 0.5 * acceleration * Math.pow(totalTime - time, 2);
    }

    public double getTotalTime(){
        return  totalTime;
    }

}
