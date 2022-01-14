package org.firstinspires.ftc.teamcode.lib.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Odometry extends TwoTrackingWheelLocalizer {
    private final DoubleSupplier[] m_encoders;
    private final DoubleSupplier m_headingSupplier;

    public Odometry(DoubleSupplier[] encoders, DoubleSupplier headingSupplier) {
        super(Arrays.asList(
                new Pose2d(0, -0.1295, 0),
                new Pose2d(0, 0.1295, 0)
        ));
        m_encoders = encoders;
        m_headingSupplier = headingSupplier;
    }


    @Override
    public double getHeading() {
        return m_headingSupplier.getAsDouble();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return  Arrays.asList(m_encoders[0].getAsDouble(), m_encoders[1].getAsDouble());
    }
}
