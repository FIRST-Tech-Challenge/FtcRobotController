package org.firstinspires.ftc.teamcode.utils;

public class Vector {
    public final float x;
    public final float y;

    public Vector(final float x, final float y) {
        this.x = x;
        this.y = y;
    }

    public Vector zero() {
        return new Vector(0, 0);
    }

    public Vector rotate(final float theta) {
        final float sinTheta = (float) Math.sin(theta);
        final float cosTheta = (float) Math.cos(theta);

        final float newX = x * cosTheta - y * sinTheta;
        final float newY = x * sinTheta + y * cosTheta;

        return new Vector(newX, newY);
    }

    public float norm(){
        return (float) Math.sqrt(x * x + y * y);
    }
}
