package org.firstinspires.ftc.teamcode.odometry;


import org.firstinspires.ftc.teamcode.utility.Average;
import org.firstinspires.ftc.teamcode.utility.Timing;
import org.firstinspires.ftc.teamcode.utility.point;
import org.firstinspires.ftc.teamcode.utility.pose;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class Odometry {
    // constants
    final static double facingForward = Math.PI/2;
    final static double facingRight = 0;

    // Thread state
    volatile boolean running = false;

    // settings
    List<OdometryWheel> wheels;
    double xCenterOfRotation = 0;
    double yCenterOfRotation = 0;
    volatile pose position;

    public void start(){
        running = true;
        loop.start();
    }
    public void end(){
        running = false;
    }

    public pose getPosition(){ return position;}

    public void resetPosition(){ position = new pose(0,0,0);}

    private final Thread loop = new Thread(() -> {
        while(running){
            wheels.forEach(OdometryWheel::updateDelta);
            position.translateRelative(curvedTrajectoryTranslation(getDeltaPose()));
            Timing.delay(1);
        }
    });

    public Odometry(pose initial, List<OdometryWheel> wheels){
        this.position = initial; this.wheels = wheels;
    }
    public Odometry(List<OdometryWheel> wheels){
        this(new pose(0,0,0), wheels);
    }

    public pose getDeltaPose(){
        Average average = new Average();

        //average vertical translation
        double vertTransNet = average.ofAll(wheels, (Function<OdometryWheel, Double>) wheel ->
            wheel.distanceTraveledTowardsAngle(
                    wheel.getDeltaPosition(), facingForward));

        //average horizontal translation
        double horoTransNet = average.ofAll(wheels, (Function<OdometryWheel, Double>) wheel ->
            wheel.distanceTraveledTowardsAngle(
                    wheel.getDeltaPosition(), facingRight));

        //average rotation
        double rotAngNet = average.ofAll(wheels, (Consumer<OdometryWheel>) wheel ->
            average.add(
                wheel.odoDeltaToBotAngle(
                    wheel.getDeltaPosition()
                        - wheel.dotProduct(horoTransNet, facingRight)
                        - wheel.dotProduct(vertTransNet, facingForward),
                    xCenterOfRotation, yCenterOfRotation),
                //The farther, the heavier.  The more aligned to direction, the heavier
                wheel.distanceToCenter() * Math.abs(wheel.dotProduct(
                                1,
                                wheel.ccTangentDir(xCenterOfRotation, yCenterOfRotation)))));

        return new pose(horoTransNet, vertTransNet, rotAngNet);
    }

    /**
     * Given the total forward translation, total sideways translation, and
     * total changed heading, and assuming that all three values change at a constant rate,
     * calculate the final change in position.
     * The calculus is tedious, but it works out nicely conceptually: imagine a line
     * from (x,y) to (x', y') and curve it into a sector of angle theta
     * @return New curved pose
     */
    public pose curvedTrajectoryTranslation(pose in){
        // if no rotation, then trig functions will be undefined
        if(in.r == 0) return in;

        // define circle that contains the sector
        double arclength = Math.hypot(in.x, in.y);
        double radius = arclength/in.r;

        //curve the path around the sector
        point curved = new point(radius * Math.cos(in.r) - radius, radius * Math.sin(in.r));

        // maintain original direction of path
        point rotated = curved.rotate(-Math.PI/2 + Math.atan2(in.y, in.x));

        return new pose(rotated.x, rotated.y, in.r);
    }
}
