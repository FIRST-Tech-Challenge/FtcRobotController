package org.firstinspires.ftc.teamcode.Reno;

public class FieldTile {
    public double destX;
    public double destY;
    public double destZ;

    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;
    public static final float oneTile   = 24 * mmPerInch;
    public static final float twoAndHalfTile   = 60 * mmPerInch;

    public FieldTile(double x, double y, double z)
    {
        destX = x;
        destY = y;
        destZ = z;
    }

}
