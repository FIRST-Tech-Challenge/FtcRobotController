package org.firstinspires.ftc.teamcode.Core.BaseClasses;

import com.arcrobotics.ftclib.geometry.Rotation2d;

public interface EctoGyroscope{

    // Gyro
    public double getHeading();

    public double getAbsoluteHeading();

    public void invertGyro();

    public double[] getAngles();

    public Rotation2d getRotation2d();

    public void reset();

}

