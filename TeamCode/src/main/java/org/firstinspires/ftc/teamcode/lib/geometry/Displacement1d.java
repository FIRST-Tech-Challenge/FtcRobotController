package org.firstinspires.ftc.teamcode.lib.geometry;

import org.firstinspires.ftc.teamcode.lib.util.Util;

import java.text.DecimalFormat;

public class Displacement1d implements State<Displacement1d> {

    protected final double displacement_;

    public Displacement1d() {
        displacement_ = 0.0;
    }

    public Displacement1d(double displacement) {
        displacement_ = displacement;
    }

    public double x() {
        return displacement_;
    }

    @Override
    public Displacement1d interpolate(final Displacement1d other, double x) {
        return new Displacement1d(Util.interpolate(displacement_, other.displacement_, x));
    }

    @Override
    public double distance(final Displacement1d other) {
        return Math.abs(x() - other.x());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Displacement1d)) {
            return false;
        }

        return Util.epsilonEquals(x(), ((Displacement1d) other).x());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format("(" + x() + ")");
    }
}
