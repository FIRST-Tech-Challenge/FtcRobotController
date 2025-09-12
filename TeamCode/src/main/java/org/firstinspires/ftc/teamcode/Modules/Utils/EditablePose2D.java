package org.firstinspires.ftc.teamcode.Modules.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// ------ READY TO TRANSFER ------ //
// this needs like no changes ever
public class EditablePose2D {

    private double x;
    private double y;
    private double h; // h Will always be in radians.

    private final DistanceUnit distanceUnit;

    public EditablePose2D(double x, double y, double h, DistanceUnit distanceUnit) {
        this.x = x;
        this.y = y;
        this.h = h;

        this.distanceUnit = distanceUnit;
    }

    public double getX (DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, x);
    }

    public double getY(DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, y);
    }

    public double getH() {
        return this.h;
    }

    public DistanceUnit getDistanceUnit() {return this.distanceUnit;}

    public void setX(double x, DistanceUnit unit) {
        this.x = this.distanceUnit.fromUnit(unit, x);
    }

    public void setY(double y, DistanceUnit unit) {
        this.y = this.distanceUnit.fromUnit(unit, y);
    }

    public void setH(double h) {
        this.h = h;
    }

    public double[] getPos() {return new double[]{x, y};}

    public boolean equals(EditablePose2D other) {
        return x == other.getX(distanceUnit) && y == other.getY(distanceUnit) && h == other.getH();
    }

    @Override
    public String toString() {
        return "POSE2D{" +
                "x=" + getX(DistanceUnit.INCH) + " cm, " +
                "y=" + getY(DistanceUnit.INCH) + " cm, " +
                "heading=" + Math.toDegrees(getH()) + "°, " +
                '}';
    }

}
