package org.firstinspires.ftc.teamcode.pathgen;

import org.firstinspires.ftc.teamcode.utility.RotationUtil;
import org.firstinspires.ftc.teamcode.utility.point;


import static java.lang.Math.*;

public class PathPoint extends point {
    PathPoint prev;
    PathPoint next;

    //control points do not move
    boolean isControl = false;
    boolean isLast = false;

    double dirVel = 0;
    double xVel = 0;
    double yVel = 0;

    public double speed;
    public double dir = 0;

    public PathPoint(double x, double y) {
        super(x, y);
    }

    public double ang(){

        double prevAng = angTo(prev);
        double nextAng = angTo(next);

        //System.out.println("ang is "+ (Math.PI - Math.abs(RotationUtil.turnLeftOrRight(prevAng, nextAng, Math.PI * 2))));
        return RotationUtil.turnLeftOrRight(prevAng, nextAng, Math.PI * 2);
    }
    void impulse(double angle, double step){
        xVel += Math.cos(angle) * step;
        yVel += Math.sin(angle) * step;
    }
    void move(double multiplier){
        if (!isControl) {
            x += xVel * multiplier;
            y += yVel * multiplier;
        }
        if(!isLast)
            dir += dirVel * multiplier;

        xVel = 0;
        yVel = 0;
        dirVel = 0;
    }

    void velocities(){
        //rotation sign
        double dirNext = Math.signum(ang());
        double dirPrev = -dirNext;

        //tangent direction
        dirNext = angTo(next) + dirNext * (Math.PI/2);
        dirPrev = angTo(prev) + dirPrev * (Math.PI/2);

        double bentness = Math.PI - abs(ang());

        //smoothen
        next.impulse(dirNext, bentness * 0.02 * distTo(next));
        prev.impulse(dirPrev, bentness * 0.02 * distTo(prev));

        //minimize distance
        impulse(angTo(next), 0.04 * distTo(next));
        impulse(angTo(prev), 0.04 * distTo(prev));

        //direction
        dirVel += 0.09 * RotationUtil.turnLeftOrRight(dir, next.dir, Math.PI * 2)
                * 10 / distTo(next);
        dirVel += 0.09 * RotationUtil.turnLeftOrRight(dir, prev.dir, Math.PI * 2)
                * 10 / distTo(prev);
        dirVel += 0.01 * RotationUtil.turnLeftOrRight(dir, angTo(next), Math.PI * 2);
    }

    double avgRotation(){
        return abs(ang());// / (distTo(prev) + distTo(next));
    }

    double angTo(point to){
        return Math.atan2(to.y-y, to.x-x);
    }

    public double distTo(point to){
        return Math.hypot(to.y-y, to.x-x);
    }

    public static PathPoint midPoint(point a, point b){
        return new PathPoint((a.x+b.x)/2, (a.y+b.y)/2);
    }

    public PathPoint translate(PathPoint p){
        this.x += p.x;
        this.y += p.y;
        return this;
    }

    public static PathPoint rotate(double x, double y, double r){
        return new PathPoint(
                x * cos(r) - y * sin(r),
                y * cos(r) + x * sin(r));
    }
    public PathPoint rotate(double r){
        return rotate(x, y, r);
    }
}
