package org.firstinspires.ftc.teamcode.mmooover;

import org.jetbrains.annotations.NotNull;

import java.util.function.Function;

public class Ramps {
    public final Function<Double, Double> begin;
    public final Function<Double, Double> end;
    public final LimitMode limitMode;

    public static final Function<Double, Double> NOP = d -> 1.0;
    public static Function<Double, Double> linear(double m) {
        return d -> m * d;
    }
    public static Function<Double, Double> power(double pow, double across) {
        return d -> Math.pow(d / across, pow);
    }

    private static double clamp(double a, double min, double max) {
        return Math.max(Math.min(a, max), min);
    }

    private static Function<Double, Double> wrapClamp(Function<Double, Double> around) {
        return d -> clamp(around.apply(d), 0.0, 1.0);
    }

    public Ramps(Function<Double, Double> begin, Function<Double, Double> end) {
        this(begin, end, LimitMode.CLIP, true);
    }

    public Ramps(Function<Double, Double> begin, Function<Double, Double> end, @NotNull LimitMode limitMode) {
        this(begin, end, limitMode, true);
    }

    public Ramps(Function<Double, Double> begin, Function<Double, Double> end, @NotNull LimitMode limitMode, boolean wrap) {
        boolean wrapBegin = wrap;
        boolean wrapEnd = wrap;
        if (begin == null) {
            begin = NOP;
            wrapBegin = false;
        }
        if (end == null) {
            end = NOP;
            wrapEnd = false;
        }
        if (wrapBegin) this.begin = wrapClamp(begin);
        else this.begin = begin;
        if (wrapEnd) this.end = wrapClamp(end);
        else this.end = end;
        this.limitMode = limitMode;
    }

    public enum LimitMode {
        CLIP,
        SCALE
    }

    public double ease(double beginVal, double endVal, double limit) {
        switch (limitMode) {
            case CLIP:
                return Math.min(begin.apply(beginVal), Math.min(end.apply(endVal), limit));
            case SCALE:
                return Math.min(begin.apply(beginVal), end.apply(endVal)) * limit;
        }
        throw new IllegalStateException("bad limit mode");
    }
}
