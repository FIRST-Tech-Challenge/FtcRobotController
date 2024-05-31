package org.firstinspires.ftc.teamcode.org.rustlib.drive;

import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Vector2d;

public class Field {
    public static final double fieldLengthIn = 141.167;
    public static double tileLengthIn = 23.4270833;
    public static final Vector2d topLeftCorner = new Vector2d(0, fieldLengthIn);
    public static final Vector2d topRightCorner = new Vector2d(fieldLengthIn, fieldLengthIn);
    public static final Vector2d bottomRightCorner = new Vector2d(fieldLengthIn, 0);
    public static final Vector2d bottomLeftCorner = new Vector2d();
    public static final Vector2d center = new Vector2d(fieldLengthIn / 2, fieldLengthIn / 2);
}
