package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

// ----- READY TO TRANSFER ----- //

public class Waypoint extends EditablePose2D {
    private boolean isAnchor;
    private double goalVelocity; // IN per second
    private double maxAccel;

    private types type;
    public enum types {
        START,
        GENERAL,
        INTERRUPTED,
        POINTTURN,
        END
    }

    public Waypoint() {
        super(0, 0, Math.toRadians(90), DistanceUnit.CM);
        type = types.GENERAL;
        isAnchor = false;
        goalVelocity = 0.0;
    }


    public Waypoint(double x, double y, double headingINDEGREES, types type, boolean isAnchor, double goalVelocity, double maxAccel, DistanceUnit distanceUnit) {
        super(x, y, Math.toRadians(headingINDEGREES), distanceUnit);
        this.type = type;
        this.isAnchor = isAnchor;
        this.goalVelocity = goalVelocity;
        this.maxAccel = maxAccel;
    }

    public Waypoint(double x, double y, double headingINDEGREES, double goalVelocity, double maxAccel, DistanceUnit units){
        this(x, y, headingINDEGREES, types.GENERAL, false, goalVelocity, maxAccel, units);
    }


    public Waypoint(double[] pos) {
        super(pos[0], pos[1], Math.toRadians(90), DistanceUnit.CM);
        type = types.GENERAL;
        isAnchor = false;
        goalVelocity = 0.0;
    }

    public types getType() {
        return type;
    }

    public void setType(types type) {
        this.type = type;
    }


    public double[] getPos() {
        return new double[]{this.getX(DistanceUnit.CM), this.getY(DistanceUnit.CM)};
    }

    public boolean isAnchor() { return isAnchor; }

    public void setAnchor() {isAnchor = true; }

    public void setAnchor(boolean anchor) {isAnchor = anchor; }

    public double getGoalVelocity() {
        return goalVelocity;
    }

    public double getMaxAccel() {return maxAccel;}

    public void setGoalVelocity(double goalVelocity) {
        this.goalVelocity = goalVelocity;
    }

    @Override
    public String toString() {
        return "Waypoint{" +
                "x=" + getX(DistanceUnit.INCH) + " cm, " +
                "y=" + getY(DistanceUnit.INCH) + " cm, " +
                "heading=" + Math.toDegrees(getH()) + "°, " +
                "type=" + type + ", " +
                "isAnchor=" + isAnchor + ", " +
                "goalVelocity=" + goalVelocity + " in/sec, " +
                "maxAccel=" + maxAccel + " in/sec²" +
                '}';
    }

}