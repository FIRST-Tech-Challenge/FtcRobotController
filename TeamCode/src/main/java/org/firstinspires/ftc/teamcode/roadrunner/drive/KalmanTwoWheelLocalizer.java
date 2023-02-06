package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.rogue.blacksmith.util.kalman.KalmanFilter;

import java.util.Arrays;
import java.util.List;

/**
 * Improve localization through running each element of localization data through Kalman Filters
 * Author: TLindauer
 */
public class KalmanTwoWheelLocalizer extends TwoWheelTrackingLocalizer {
    private final KalmanFilter
        headingFilter,
        wheelPos1Filter,
        wheelPos2Filter,
        headingVelocityFilter,
        wheelPos1VelocityFilter,
        wheelPos2VelocityFilter;

    public KalmanTwoWheelLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(hardwareMap, drive);

        headingFilter = new KalmanFilter(0.25, 0.125); // Remember heading is in radians
        headingVelocityFilter = new KalmanFilter(0.500, 0.225);
        wheelPos1Filter = new KalmanFilter(9, 11);
        wheelPos2Filter = new KalmanFilter(9, 11);

        wheelPos1VelocityFilter = new KalmanFilter(8, 7);
        wheelPos2VelocityFilter = new KalmanFilter(8, 7);
    }

    @Override
    public double getHeading() {
        return headingFilter.filter(super.getHeading());
    }

    @Override
    public Double getHeadingVelocity() {
        Double in = super.getHeadingVelocity();
        if (in != null)
            return headingVelocityFilter.filter(in);
        return 0.0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> in = super.getWheelPositions();
        return Arrays.asList(wheelPos1Filter.filter(in.get(0)), wheelPos2Filter.filter(in.get(1)));
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        List<Double> in = super.getWheelVelocities();
        return Arrays.asList(wheelPos1VelocityFilter.filter(in.get(0)), wheelPos2VelocityFilter.filter(in.get(1)));
    }
}
