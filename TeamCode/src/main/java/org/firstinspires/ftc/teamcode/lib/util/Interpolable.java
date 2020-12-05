package org.firstinspires.ftc.teamcode.lib.util;

public interface Interpolable<T> {
    T interpolate(T other, double x);
}
