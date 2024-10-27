package org.firstinspires.ftc.teamcode.utils.controller;

import java.util.function.Function;

public enum PowerCurve {
    Linear(x -> x),
    Quadratic(x -> Math.abs(x) * x),
    Cubic(x -> x * x * x);

    private final Function<Float, Float> curveFunction;
    PowerCurve(Function<Float, Float> curveFunction) {
        this.curveFunction = curveFunction;
    }

    public float apply(float f) {
        return curveFunction.apply(f);
    }
}
