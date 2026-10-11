package org.firstinspires.ftc.teamcode;

// keeping the robot pointed in a perfectly straight line or snapping to a set angle while the driver translates and weaves around opponents
public class DriverAssist {

    private double hGain; // how aggressively the robot corrects its angle when bumped
    private double maxAV; // top spin speed your robot is physically capable of achieving (angular velocity)
    private double headingAuthority; // if set to 0.3, 30% of your motor power goes to tracking heading, leaving 70% for driver movement
    private Double lockedHeading = null; // the driver has full manual turning control if null

    public DriverAssist(double iQ, double iR, double iDt, double iMaxAV, double iHA) {
        if (iHA <= 0 || iHA >= 1) {
            throw new IllegalArgumentException("headingAuthority must be between 0 and 1 (exclusive)");
        }
        if (iMaxAV <= 0) {
            throw new IllegalArgumentException("maxAngularVelocity must be greater than 0");
        }

        // to calculate and save the LQR heading gain put input variables into the LQR math formula
        hGain = LQRPathFollower.scalarLqrGain(iDt, iQ, iR);
        maxAV = iMaxAV;
        headingAuthority = iHA;
    }

    public static DriverAssist withDefaults(double maxAV) {
        // sets standard defaults as 20ms loop speed + reserves 30% of power budget for heading correction
        return new DriverAssist(8.0, 1.0, 0.02, maxAV, 0.30);
    }

    public void lockHeading(double radians) {
        // lock onto a field angle (like 0 radians for facing the backdrop)
        // You would typically bind this method to a controller button press
        lockedHeading = LQRPathFollower.normalizeAngle(radians);
    }

    public void snapToNearest(double hd, int divs) {
        if (divs < 1) {
            System.err.println("ERROR: divs must be 1 or greater");
            return;
        }
        // Calculate the size of each angle step
        double step = (2 * Math.PI) / divs;
        // Find how many steps fit into our current angle (and round it)
        long roundedSteps = Math.round(hd / step);
        // Multiply back to get the closest exact step angle
        double rawAngle = roundedSteps * step;
        // Clean up the angle so it stays between -PI and PI
        lockedHeading = LQRPathFollower.normalizeAngle(rawAngle);
    }

    // break the angle lock or check if there is a lock
    public void release() {
        lockedHeading = null;
    }

    public boolean isLocked() {
        return lockedHeading != null;
    }

    public Double lockedHeading() {
        return lockedHeading;
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////// Cubic Bezier Curve timeeee ///////////////////
    //////////////////////////////////////////////////////////////////////
    ///////////////// these r so much cooler than * / right? /////////////
    //////////////////////////////////////////////////////////////////////

    public double[] update(double stickX, double stickY, double stickTurn, double heading) {
        // This sets up method params + defines two empty vars:
        // rotation (how fast the robot will spin)
        // translationScale (how much top speed the driver is allowed to use)
        double r;
        double tScale;

        // If the angle lock is off the robot behaves normal
        // Turning power matches the driver's right joystick (stickTurn)
        // and the driver retains 100% top speed (1.0)
        if (lockedHeading == null) {
            r = stickTurn;
            tScale = 1.0;
        }
        // If the angle lock is on then the code ignores the driver's right joystick
        // Instead it looks at the target angle vs your actual gyro angle to find the error (err)
        // It multiplies this by a tuning constant (headingGain) to get a target spin speed
        // translates that into a -1.0 to 1.0 power scale
        // and clips it so it never uses more power than your reserved budget (headingAuthority)
        else {
            double err = LQRPathFollower.normalizeAngle(lockedHeading - heading);
            double omega = hGain * err;
            // heading is counterclockwise positive but a positive r in the mixer below turns the robot CLOCKWISE,
            // so the correction has to be negated or the lock pushes away from the target and the robot spins/jitters
            r = -omega / maxAV;
            r = Math.max(-headingAuthority, Math.min(headingAuthority, r));

            // this reserves the power frfr rest is js there
            tScale = 1.0 - headingAuthority;
        }

        // Robot-Centric Driving Layout
        // stickY is the forward/back stick so it drives vx (all four wheels the same way),
        // stickX is the left/right stick so it drives vy (strafe). These were swapped before.
        double vx = stickY * tScale;
        double vy = stickX * tScale;

        double[] p = new double[]{
                vx + vy + r,
                vx - vy - r,
                vx - vy + r,
                vx + vy - r
        };

        // Proportional scale reducer normalization logic
        double rawMax = 0;
        for (double v : p) {
            rawMax = Math.max(rawMax, Math.abs(v));
        }

        double d = Math.max(1.0, rawMax);
        for (int i = 0; i < 4; i++) {
            p[i] /= d;
        }
        return p;
    }

    public static HolonomicPath scoreMacroPath(double curX, double curY, double curHeading, double tX, double tY, double tHeading) {
        double dist = Math.hypot(tX - curX, tY - curY);
        double handle = Math.max(4.0, dist * 0.4);

        // with no empty overrides blocking the internal arithmetic engine!
        return new CubicBezierPath(
                 new Vec2(curX, curY), // Start Point
                 new Vec2(curX + Math.cos(curHeading) * handle, curY + Math.sin(curHeading) * handle), // Exit Guide Vector
                 new Vec2(tX - Math.cos(tHeading) * handle, tY - Math.sin(tHeading) * handle), // Entry Guide Vector
                 new Vec2(tX, tY) // Scoring Destination Target
         );
    }
}