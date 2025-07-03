package org.firstinspires.ftc.teamcode.utils;

public class Angle {
    public static final float pi = (float) Math.PI;
    public static final float twoPi = (float) Math.PI * 2;

    public static float wrapPlusMinusPI(final float theta) {
        final float wrapped = theta % twoPi;

        if (wrapped > pi) {
            return wrapped - twoPi;
        } else if (wrapped < -pi) {
            return wrapped + twoPi;
        }

        return wrapped;

    }
}
