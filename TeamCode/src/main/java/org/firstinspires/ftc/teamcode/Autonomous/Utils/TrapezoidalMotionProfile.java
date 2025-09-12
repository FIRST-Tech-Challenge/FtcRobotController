package org.firstinspires.ftc.teamcode.Autonomous.Utils;

/**
 * Creates a trapezoidal motion profile for 1 dimension.
 * Usage guide in the documentation on discord
 */
public class TrapezoidalMotionProfile {

    private double maxA; // Maximum acceleration
    private double maxV; // Maximum velocity

    // Woah lookie here at Aaron's AI documentation o.o
    // Although I could have used more documentation so I wrote in a lot more.
    // All timing/distance values are now computed using the absolute value of the distance,
    // and then we reapply the sign at the end.
    private double dta;        // Time to reach max velocity (maxV) (always positive)
    private double effMaxV;    // Effective max velocity (can be lower than maxV if distance is short)
    private double accelDist;  // Distance covered during acceleration (absolute value)
    private double cruiseDist; // Distance covered during cruising (absolute value)
    private double cruiseTime; // Time spent cruising (always positive)
    private double decelTime;  // Start time of deceleration phase (always positive)
    private double totalTime;  // Total time for the entire motion profile (always positive)
    private double direction;  // +1 for forward, -1 for backward

    public TrapezoidalMotionProfile(double maxA, double maxV) {
        this.maxA = maxA;
        this.maxV = maxV;
    }

    public void setProfile(double mA, double mV) {
        maxA = mA;
        maxV = mV;
    }

    // Calculate motion profile based on a goal and a current position.
    // This version calculates using the absolute distance and then re-applies the direction.

    /**
     * Creates the accelerate/decelerate portions of the path given. Most Crucially, this version stores the direction of travel.
     * @param goal End distance
     * @param currPos current position
     */
    public void calculateProfile(double goal, double currPos) {
        double distance = goal - currPos;

        if (distance == 0){
            return;
        }
        direction = Math.signum(distance);  // Store the travel direction (+1 or -1)
        double absDistance = Math.abs(distance); // Work with absolute values for timing calculations

        dta = maxV / maxA; // v = at therefore t = v/a
        double halfDistance = absDistance / 2; // test var
        accelDist = 0.5 * maxA * Math.pow(dta, 2); // ∆x = 1/2 a∆t^2 (absolute value)

        effMaxV = maxV; // Will stay this way unless proven could be better (Yeah...)
        if (accelDist > halfDistance) { // In case accelDist is too big. (Cuz the largest it can be is right at the midpoint.)
            dta = Math.sqrt(2 * halfDistance / maxA); // fix dta in this case (otherwise it's not gon' work)
            effMaxV = dta * maxA; // fix effMaxV (Just because it'll help-) (yeah, yes.)
            accelDist = 0.5 * maxA * Math.pow(dta, 2); // re-solve accelDist with new dta
        }

        cruiseDist = absDistance - 2 * accelDist; // Calculate the distance the robot spends at a constant velocity (cruiseDist)
        cruiseTime = cruiseDist / effMaxV; // Since effMaxV is the max in this profile. And cuz x = vt therefore t = x/v.
        decelTime = dta + cruiseTime; // decelTime is WHEN it starts decelerating. That's after acceleration and cruising.
        totalTime = 2 * dta + cruiseTime; // or dta + cruiseTime + dta (Honestly who gives a fuck so)

        //
        //         ______________
        //        /|            |\
        //       / |            | \
        //      /__|____________|__\
        //
        // We all love the trapezoid drawing -_-
        //
    }

    // Convenience overload if you just have a distance (assumes starting at 0)
    public void calculateProfile(double distance) {
        calculateProfile(distance, 0);
    }

    /**
     * @param time current time, in seconds
     * @return expectedPosition from Profile
     */
    public double getExpectedPosition(double time) {
        double pos; // computed position (absolute value)

        // If time exceeds the total duration, return the target distance (absolute value)
        if (time > totalTime) {
            pos = 2 * accelDist + cruiseDist; // This is the full distance
        }
        // return expected position depending on the phase
        else if (time < dta) { // Acceleration phase
            pos = 0.5 * maxA * Math.pow(time, 2); // ∆x = 0.5at^2
        } else if (time < decelTime) { // Cruise phase (decelTime is WHEN deceleration starts)
            pos = accelDist + effMaxV * (time - dta); // The triangle plus the rectangle (draw the trapezoid if ur confused (in ur head lol))
        } else { // Deceleration phase
            pos = accelDist + cruiseDist + effMaxV * (time - decelTime) - 0.5 * maxA * Math.pow(time - decelTime, 2); // Draw it
        }

        // Reapply the original direction (so we get negative positions when going backward)
        return direction * pos;

        //
        //       _________________________
        //      /|                   |\  | look at this beautiful finished rectangle on the right
        //     / |                   | \ | of this unfinished trapezoid
        //    /  |                   |  \| I am obsessed with diagrams I know
        //   /   |                   |   | And ihdgaf (I honestly don't give a fuck).
        //  /    |                   |   |
        // /_____|___________________|___|
        //
    }
}
