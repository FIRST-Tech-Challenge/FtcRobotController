package org.firstinspires.ftc.teamcode.lib.util;

import org.firstinspires.ftc.teamcode.lib.geometry.Twist2d;

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average of the Twist2d class
 */
public class MovingAverageTwist2d {
    ArrayList<Twist2d> twists = new ArrayList<Twist2d>();
    private int maxSize;

    public MovingAverageTwist2d(int maxSize) {
        this.maxSize = maxSize;
    }

    public synchronized void add(Twist2d twist) {
        twists.add(twist);
        if (twists.size() > maxSize) {
            twists.remove(0);
        }
    }

    public synchronized Twist2d getAverage() {
        double x = 0.0, y = 0.0, t = 0.0;

        for (Twist2d twist : twists) {
            x += twist.dx;
            y += twist.dy;
            t += twist.dtheta;
        }

        double size = getSize();
        return new Twist2d(x / size, y / size, t / size);
    }

    public int getSize() {
        return twists.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        twists.clear();
    }

}