package org.firstinspires.ftc.teamcode.Swerve.Drive.ParticleFilter;

import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import java.util.ArrayList;

public class MonteCarloSwerveFilter { //todo standardize units across swerve
    private final ArrayList<SwerveParticle> particles = new ArrayList<>();
    private final int PARTICLE_COUNT = 100;

    private final double ROBOT_MASS = 8.16; //todo get more precise measurement
    private final double ROT_INERTIA = 0.5; //todo find a good way to measure this

    private final double dX = (9.927 / 2.0) * 0.0254;
    private final double dY = (9.921 / 2.0) * 0.0254;
    private final double[] mX = {dX, dX, -dX, -dX};
    private final double[] mY = {dY, -dY, dY, -dY};

    public MonteCarloSwerveFilter() {
        for (int i = 0; i < PARTICLE_COUNT; i++) {
            particles.add(new SwerveParticle());
        }
    }

    public void update(double[] targetPowers, double[] targetAngles, double[] wheelVel, Pose actualVel, Pose prevVel, double dt, double voltage) {
        if (Math.abs(actualVel.x) < 0.01 && Math.abs(actualVel.y) < 0.01 && Math.abs(actualVel.heading) < 0.01) return;

        for (SwerveParticle p : particles) {
            Pose pred = calculatePredictedVelocity(targetPowers, targetAngles, p.offsets, wheelVel, prevVel, dt, voltage);

            double error = Math.pow(pred.x - actualVel.x, 2) +
                    Math.pow(pred.y - actualVel.y, 2) +
                    Math.pow(pred.heading - actualVel.heading, 2) * 2.5;

            p.weight = 1.0 / (error > 0 ? error : 0.0001);
        }

        resample();
    }

    private Pose calculatePredictedVelocity(double[] powers, double[] angles, double[] offsets, double[] wheelVel, Pose prevVel, double dt, double voltage) {
        double voltagecompensation = voltage/12.4;
        double fx = 0, fy = 0, tau = 0;
        double maxForce = 40.0; //todo get real number
        double maxVel = 2.5; //todo get real number

        for (int i = 0; i < 4; i++) {
            if (powers[i] == 0) continue;

            double realAngle = angles[i] + offsets[i];
            double f = powers[i] * voltagecompensation * maxForce * (1.0 - (Math.abs(wheelVel[i]) / maxVel));

            double fxi = f * Math.cos(realAngle);
            double fyi = f * Math.sin(realAngle);

            fx += fxi;
            fy += fyi;
            tau += (mX[i] * fyi) - (mY[i] * fxi);
        }

        return new Pose(
                prevVel.x + (fx / ROBOT_MASS) * dt,
                prevVel.y + (fy / ROBOT_MASS) * dt,
                prevVel.heading + (tau / ROT_INERTIA) * dt
        );
    }

    private void resample() {
        particles.sort((p1, p2) -> Double.compare(p2.weight, p1.weight));

        int genSize = PARTICLE_COUNT / 5;
        for (int i = genSize; i < PARTICLE_COUNT; i++) {
            SwerveParticle parent = particles.get(i % genSize);
            particles.get(i).copyFrom(parent);

            particles.get(i).mutate(Math.toRadians(0.5));
        }
    }

    public double[] getBestOffsets() {
        return particles.get(0).offsets;
    }
}