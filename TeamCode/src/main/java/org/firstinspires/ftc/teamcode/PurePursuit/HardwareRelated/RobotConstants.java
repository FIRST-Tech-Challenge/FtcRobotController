package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

public class RobotConstants {

    public static double

        /*-- Localization --*/
        TICKS_PER_REV = 4096,
        WHEEL_RADIUS = 0.688976378, // in

        parYTicks = -4376.102487991054521764364486425, // y position of the parallel encoder (in tick units)
        perpXTicks = -5535.1782821076040977992502152618, // x position of the perpendicular encoder (in tick units)

        /*-- Robot Movement --*/
        maxVelocity = 50, // max target velocity for the path follow (ticks per second)
        maxAcceleration = 50, // max target acceleration and deceleration for the path follow (ticks per second)

        maxRotationalVelocity = 180, // max rotational target velocity for the path follow
        maxRotationalAcceleration = 180, // max rotational target acceleration and deceleration for the path follow

        minRadiusRange = 10, // min lookahead distance (inches)
        maxRadiusRange = 30, // max lookahead distance (inches)

        radiusMutliplier = 1, // multiplier of the dynamic radius

        robotX = 15, // robot's size in the x axis
        robotY = 15; // robot's size in the y axis

    /**
     * Calculates the inches per encoder tick
     * @return
     */
    public static double inPerTick_Calculation() {
        return 2 * WHEEL_RADIUS * Math.PI / TICKS_PER_REV;
    }
}