package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class OverflowEncoder implements Encoder {
    // encoder velocities are sent as 16-bit ints
    // by the time they reach here, they are widened into an int and possibly negated
    private static final int CPS_STEP = 0x10000;

    private static int inverseOverflow(int input, double estimate) {
        // convert to uint16
        int real = input & 0xFFFF;
        // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
        // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
        real += ((real % 20) / 4) * CPS_STEP;
        // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

    public final RawEncoder encoder;

    private int lastPosition;
    private final ElapsedTime lastUpdate;

    private final RollingThreeMedian velEstimate;

    public OverflowEncoder(RawEncoder e) {
        encoder = e;

        lastPosition = e.getPositionAndVelocity().position;
        lastUpdate = new ElapsedTime();

        velEstimate = new RollingThreeMedian();
    }

    @Override
    public PositionVelocityPair getPositionAndVelocity() {
        PositionVelocityPair p = encoder.getPositionAndVelocity();
        double dt = lastUpdate.seconds();
        double v = velEstimate.update((p.position - lastPosition) / dt);

        lastPosition = p.position;
        lastUpdate.reset();

        return new PositionVelocityPair(
                p.position,
                inverseOverflow(p.velocity, v)
        );
    }

    @Override
    public DcMotorController getController() {
        return encoder.getController();
    }
}
