package org.firstinspires.ftc.teamcode.Swerve.Drive.ParticleFilter;

import androidx.annotation.NonNull;

public class SwerveParticle {
    public double[] offsets = new double[4];
    public double weight = 1.0;

    public SwerveParticle() {
        // Initialize once
        for (int i = 0; i < 4; i++) {
            offsets[i] = (Math.random() - 0.5) * Math.toRadians(5);
        }
    }

    public void copyFrom(@NonNull SwerveParticle other) {
        System.arraycopy(other.offsets, 0, this.offsets, 0, 4);
        this.weight = other.weight;
    }

    public void mutate(double intensity) {
        for (int i = 0; i < 4; i++) {
            offsets[i] += (Math.random() - 0.5) * intensity;
        }
    }
}