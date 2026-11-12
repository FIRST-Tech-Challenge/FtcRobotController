package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import java.util.function.Consumer;

/** Pedro uses left/CCW positive. Mix once, bypassing TeleOp's strafe multiplier. */
public final class PedroMecanumAdapter implements Drivetrain {
    private final Consumer<double[]> output;
    private final double powerLimit;

    public PedroMecanumAdapter(MecanumDrivetrain drive) {
        this(drive, PedroConfiguration.MAX_WHEEL_POWER);
    }
    /** Pedro tuning only: the Foresight tuner identifies full-power dynamics. */
    PedroMecanumAdapter(MecanumDrivetrain drive, double powerLimit) {
        this(p -> drive.setWheelPowers(p[0], p[1], p[2], p[3]), powerLimit);
    }
    PedroMecanumAdapter(Consumer<double[]> output, double powerLimit) {
        this.output = output; this.powerLimit = powerLimit;
    }
    static double[] wheels(DrivePowers p) {
        return new double[]{p.forward() - p.strafe() - p.turn(), p.forward() + p.strafe() + p.turn(),
                p.forward() + p.strafe() - p.turn(), p.forward() - p.strafe() + p.turn()};
    }
    @Override public void drive(DrivePowers powers, boolean manual) {
        double[] wheels = wheels(powers);
        double scale = 1;
        for (double power : wheels) {
            if (!Double.isFinite(power)) { stop(); return; }
            scale = Math.max(scale, Math.abs(power) / powerLimit);
        }
        for (int i = 0; i < wheels.length; i++) wheels[i] /= scale;
        output.accept(wheels);
    }
    @Override public double maxScaling(DrivePowers current, DrivePowers delta) {
        double[] a = wheels(current), b = wheels(delta);
        double scale = 1;
        for (int i = 0; i < a.length; i++) {
            if (b[i] > 1e-9) scale = Math.min(scale, (powerLimit - a[i]) / b[i]);
            else if (b[i] < -1e-9) scale = Math.min(scale, (-powerLimit - a[i]) / b[i]);
        }
        return Math.max(0, scale);
    }
    @Override public void stop() { output.accept(new double[4]); }
}
